/*
  Teensy_FC_MPU6050_Student_Reference — Lab 8 pin-redacted reference

  NOT READY TO COMPILE OR FLASH. Only the four course ESC command pins are fixed.
  Choose the remaining pins for your own schematic and PCB; see README.md in
  Firmware_Reference. Control/calibration/tuning behavior is inherited context,
  not validated settings for your airframe. Keep propellers disconnected.

  Receiver / drone-side firmware for a custom Teensy 4.0 flight-controller PCB.

  EXPERIMENTAL / HARDWARE VALIDATION REQUIRED. This is a dRehmFlight
  controlANGLE2-tailored 2 kHz branch. It retains the source receiver,
  arming/failsafe/STOP, body frame, differential mixer and ESC wrappers; only the
  IMU estimator/filter profile and controller law are deliberately different.

  Important project-specific differences:
  - Receiver input is a custom nRF24L01+PA+LNA packet, not PWM/PPM/SBUS/DSM/iBUS.
  - IMU is an MPU6050 in I2C mode on student-selected compatible SDA/SCL pins.
    The inherited axis transform must be verified for your module orientation.
  - Right joystick press toggles arming at low throttle. Left joystick press
    requests a fresh IMU calibration while disarmed at low throttle.
  - Calibration is manual after every boot; motors cannot arm until it succeeds.
  - RC PWM debug pins are scope/demo outputs only, not internal flight inputs.
  - Example motor order is M1 D8, M2 D4, M3 D22, M4 D23; only the pin set is required.
  - This 2 kHz experimental branch supports OneShot125, DShot300, and DShot600 output.
  - Select exactly one output protocol in ReceiverProtocolConfig.h.
*/

// Leave at 0 whenever the PID tuner is in use. USB is now the tuner's protocol
// stream, and human-readable status lines interleave with the $T rows it reads.
// The tuner survives it - it logs anything that is not a protocol line - but
// the serial log becomes hard to read.
#define DEBUG_SERIAL 0

// Optional scope outputs are disabled. Their pins are intentionally unassigned;
// do not allocate them unless your board includes that optional function.
#define ENABLE_DEBUG_PWM_OUTPUTS 0

// Keep explicit arming mandatory on the V-Final receiver.
#define REQUIRE_EXPLICIT_ARM 1

#include "ReceiverProtocolConfig.h"

// This experimental 2 kHz variant requires a high-rate ESC protocol.
// Standard 1000-2000 us PWM cannot produce a valid 2 kHz command cadence.
#if ESC_PROTOCOL_PWM
#error "The 2 kHz receiver requires OneShot125 or DShot; ESC_PROTOCOL_PWM is not supported."
#endif

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RF24.h>
#include <IntervalTimer.h>
#include <math.h>
#include <string.h>

// PID Tuner Protocol v1. Serves the descriptor and live gains over this board's
// own USB port, and supplies the identity/descriptor bytes the transmitter
// relays over the radio. See PROTOCOL.md in the tuner suite.
// Keep MPU6050 residual calibration separate from BMI160. The fresh slot keeps
// saved offsets from the former nose direction out of this body frame. The
// Legacy MPU builds share this EEPROM slot only after a fresh calibration.
// Body-frame residuals stored at 384 were learned with the old sensor mapping.
// Keep them available to the preserved original build; start this mapping clean.
#define PIDLINK_EEPROM_ADDR 640
#include "NonBlockingUsb.h"
NonBlockingUsb usbProtocol(Serial);
#define PIDLINK_STREAM usbProtocol
#define PIDLINK_TX_READY(bytes) usbProtocol.canQueue(bytes)
#include "PidLink.h"
#include "FailsafeDescent.h"

#if ENABLE_DEBUG_PWM_OUTPUTS
#include <Servo.h>
#endif

// ---------------------------------------------------------------------------
// PID Tuner Protocol descriptor
//
// Served over this board's own USB port, and in 19-byte chunks over the radio
// when the tuner asks the transmitter for it. The browser caches it by content
// hash, so every byte here is significant - including the spaces inside labels.
//
// telemetry.fields must stay in the same order this sketch emits $T values AND
// the same order the TRANSMITTER prints them. Change one, change all three.
// ---------------------------------------------------------------------------
static const char DESCRIPTOR[] =
"{"
  "\"protocol\":1,"
  "\"rig\":\"drone\","
  "\"name\":\"V-Final MPU6050 2k dRehm\"," 
  "\"version\":\"5.4-vfinal-mpu-rotated\"," 
  "\"board\":\"Teensy4\","
  "\"imu\":{\"type\":\"MPU6050\",\"interface\":\"I2C\",\"address\":\"0x68/0x69\",\"rate_hz\":1000,\"read_rate_hz\":2000,\"gyro_internal_hz\":8000},"
  "\"control\":{\"mode\":\"dRehm angle\",\"rate_hz\":2000},"
  "\"capture\":{\"window_ms\":1500,\"pre_ms\":300},"
  "\"pid_trace\":{\"passive\":true,\"rate_hz\":500,\"pre_ms\":200,\"post_ms\":500,"
                  "\"controllers\":[\"roll\",\"pitch\",\"yaw\"],\"terms\":\"actual_rate_loop\"},"
  "\"features\":[\"pid_set\",\"pid_get\",\"pid_reset\",\"calibrate\",\"calibrate_persist\","
                "\"stop\",\"telemetry_rate\",\"pid_trace_passive\"],"
  "\"controllers\":["
    "{\"id\":\"roll\",\"label\":\"Roll rate\",\"params\":["
      "{\"id\":\"kp\",\"label\":\"P\",\"min\":0,\"max\":2,\"step\":0.001,\"default\":0.2},"
      "{\"id\":\"ki\",\"label\":\"I\",\"min\":0,\"max\":5,\"step\":0.001,\"default\":0.54},"
      "{\"id\":\"kd\",\"label\":\"D\",\"min\":0,\"max\":0.01,\"step\":0.00001,\"default\":0.0002}]},"
    "{\"id\":\"pitch\",\"label\":\"Pitch rate\",\"params\":["
      "{\"id\":\"kp\",\"label\":\"P\",\"min\":0,\"max\":2,\"step\":0.001,\"default\":0.32},"
      "{\"id\":\"ki\",\"label\":\"I\",\"min\":0,\"max\":5,\"step\":0.001,\"default\":0.21},"
      "{\"id\":\"kd\",\"label\":\"D\",\"min\":0,\"max\":0.01,\"step\":0.00001,\"default\":0.0002}]},"
    "{\"id\":\"yaw\",\"label\":\"Yaw rate\",\"params\":["
      "{\"id\":\"kp\",\"label\":\"P\",\"min\":0,\"max\":2,\"step\":0.001,\"default\":0.115},"
      "{\"id\":\"ki\",\"label\":\"I\",\"min\":0,\"max\":5,\"step\":0.001,\"default\":0.3},"
      "{\"id\":\"kd\",\"label\":\"D\",\"min\":0,\"max\":0.01,\"step\":0.00001,\"default\":0.00015}]},"
    "{\"id\":\"roll_angle\",\"label\":\"Roll angle (outer)\",\"params\":["
      "{\"id\":\"kp\",\"label\":\"P\",\"min\":0,\"max\":1,\"step\":0.001,\"default\":0.166667},"
      "{\"id\":\"ki\",\"label\":\"I\",\"min\":0,\"max\":0.5,\"step\":0.001,\"default\":0.023333},"
      "{\"id\":\"kd\",\"label\":\"D\",\"min\":0,\"max\":0.2,\"step\":0.0001,\"default\":0.0}]},"
    "{\"id\":\"pitch_angle\",\"label\":\"Pitch angle (outer)\",\"params\":["
      "{\"id\":\"kp\",\"label\":\"P\",\"min\":0,\"max\":1,\"step\":0.001,\"default\":0.166667},"
      "{\"id\":\"ki\",\"label\":\"I\",\"min\":0,\"max\":0.5,\"step\":0.001,\"default\":0.026667},"
      "{\"id\":\"kd\",\"label\":\"D\",\"min\":0,\"max\":0.2,\"step\":0.0001,\"default\":0.0}]},"
    "{\"id\":\"level_trim\",\"label\":\"Level trim (deg)\",\"params\":["
      "{\"id\":\"kp\",\"label\":\"Roll hover angle\",\"min\":-5,\"max\":5,\"step\":0.05,\"default\":0},"
      "{\"id\":\"ki\",\"label\":\"Pitch hover angle\",\"min\":-5,\"max\":5,\"step\":0.05,\"default\":1.3}]}],"
  "\"telemetry\":{\"rate_hz\":25,\"rates_available\":[25,50],\"fields\":["
    "{\"id\":\"t\",\"label\":\"Time\",\"unit\":\"us\",\"role\":\"time\"},"
    "{\"id\":\"gx\",\"label\":\"Roll rate\",\"unit\":\"deg/s\",\"role\":\"measurement\",\"controller\":\"roll\"},"
    "{\"id\":\"spx\",\"label\":\"Roll target\",\"unit\":\"deg/s\",\"role\":\"setpoint\",\"controller\":\"roll\",\"trigger\":40},"
    "{\"id\":\"gy\",\"label\":\"Pitch rate\",\"unit\":\"deg/s\",\"role\":\"measurement\",\"controller\":\"pitch\"},"
    "{\"id\":\"spy\",\"label\":\"Pitch target\",\"unit\":\"deg/s\",\"role\":\"setpoint\",\"controller\":\"pitch\",\"trigger\":40},"
    "{\"id\":\"gz\",\"label\":\"Yaw rate\",\"unit\":\"deg/s\",\"role\":\"measurement\",\"controller\":\"yaw\"},"
    "{\"id\":\"spz\",\"label\":\"Yaw target\",\"unit\":\"deg/s\",\"role\":\"setpoint\",\"controller\":\"yaw\",\"trigger\":40},"
    "{\"id\":\"m1\",\"label\":\"M1 rear-right\",\"unit\":\"us\",\"role\":\"output\",\"min\":1070,\"max\":2000},"
    "{\"id\":\"m2\",\"label\":\"M2 rear-left\",\"unit\":\"us\",\"role\":\"output\",\"min\":1070,\"max\":2000},"
    "{\"id\":\"m3\",\"label\":\"M3 front-right\",\"unit\":\"us\",\"role\":\"output\",\"min\":1070,\"max\":2000},"
    "{\"id\":\"m4\",\"label\":\"M4 front-left\",\"unit\":\"us\",\"role\":\"output\",\"min\":1070,\"max\":2000},"
    "{\"id\":\"sr\",\"label\":\"Roll stick\",\"role\":\"input\",\"min\":-1000,\"max\":1000},"
    "{\"id\":\"sp\",\"label\":\"Pitch stick\",\"role\":\"input\",\"min\":-1000,\"max\":1000},"
    "{\"id\":\"sy\",\"label\":\"Yaw stick\",\"role\":\"input\",\"min\":-1000,\"max\":1000},"
    "{\"id\":\"thr\",\"label\":\"Throttle\",\"unit\":\"us\",\"role\":\"input\",\"min\":1000,\"max\":2000,\"neutral\":1000,\"activity\":false},"
    "{\"id\":\"roll_deg\",\"label\":\"Roll angle\",\"unit\":\"deg\",\"role\":\"measurement\",\"controller\":\"roll_angle\"},"
    "{\"id\":\"roll_deg_sp\",\"label\":\"Roll angle target\",\"unit\":\"deg\",\"role\":\"setpoint\",\"controller\":\"roll_angle\",\"trigger\":5},"
    "{\"id\":\"pitch_deg\",\"label\":\"Pitch angle\",\"unit\":\"deg\",\"role\":\"measurement\",\"controller\":\"pitch_angle\"},"
    "{\"id\":\"pitch_deg_sp\",\"label\":\"Pitch angle target\",\"unit\":\"deg\",\"role\":\"setpoint\",\"controller\":\"pitch_angle\",\"trigger\":5},"
    "{\"id\":\"angle_mode\",\"label\":\"Angle mode\",\"role\":\"flag\",\"severity\":\"info\",\"true_label\":\"ON\",\"false_label\":\"off\"},"
    "{\"id\":\"vbat\",\"label\":\"Battery\",\"unit\":\"V\",\"role\":\"aux\"},"
    "{\"id\":\"fs\",\"label\":\"Failsafe\",\"role\":\"flag\"},"
    "{\"id\":\"lowbat\",\"label\":\"Battery low\",\"role\":\"flag\"},"
    "{\"id\":\"link\",\"label\":\"Radio hardware fault\",\"role\":\"flag\"}]},"
  "\"outputs\":["
    "{\"id\":\"m1\",\"label\":\"M1 rear-right\",\"position\":\"rear-right\",\"pin\":8,\"min\":1000,\"max\":2000,\"idle\":1070},"
    "{\"id\":\"m2\",\"label\":\"M2 rear-left\",\"position\":\"rear-left\",\"pin\":4,\"min\":1000,\"max\":2000,\"idle\":1070},"
    "{\"id\":\"m3\",\"label\":\"M3 front-right\",\"position\":\"front-right\",\"pin\":22,\"min\":1000,\"max\":2000,\"idle\":1070},"
    "{\"id\":\"m4\",\"label\":\"M4 front-left\",\"position\":\"front-left\",\"pin\":23,\"min\":1000,\"max\":2000,\"idle\":1070}],"
  "\"output_protocols\":[\"oneshot125\",\"dshot300\",\"dshot600\"]"
"}";

PidLink link;

// -----------------------------
// Pin definitions
// -----------------------------
#error "Lab 8 reference only: keep the fixed motor pins; assign and review all other pins and divider values before compiling."
constexpr uint8_t UNASSIGNED_PIN = 255;
constexpr uint8_t NRF_CE_PIN = UNASSIGNED_PIN;
constexpr uint8_t NRF_CSN_PIN = UNASSIGNED_PIN;
// Select a valid pin set for the SPI peripheral used by the radio.
// Changing SPI instance requires updating every use of SPI and the RF24 setup.
constexpr uint8_t SPI_MOSI_PIN = UNASSIGNED_PIN;
constexpr uint8_t SPI_MISO_PIN = UNASSIGNED_PIN;
constexpr uint8_t SPI_SCK_PIN = UNASSIGNED_PIN;
static_assert(NRF_CE_PIN != UNASSIGNED_PIN && NRF_CSN_PIN != UNASSIGNED_PIN &&
              SPI_MOSI_PIN != UNASSIGNED_PIN && SPI_MISO_PIN != UNASSIGNED_PIN &&
              SPI_SCK_PIN != UNASSIGNED_PIN, "Assign the radio control and compatible SPI pins");

// REQUIRED PIN SET: D4, D8, D22, D23. This is an example motor order only.
// Choose channel order to match your schematic/PCB and later physical frame map.
constexpr uint8_t MOTOR1_CMD_PIN = 8;   // Example M1
constexpr uint8_t MOTOR2_CMD_PIN = 4;   // Example M2
constexpr uint8_t MOTOR3_CMD_PIN = 22;  // Example M3
constexpr uint8_t MOTOR4_CMD_PIN = 23;  // Example M4
constexpr uint8_t LINK_LED_CTRL_PIN = UNASSIGNED_PIN;
static_assert(LINK_LED_CTRL_PIN != UNASSIGNED_PIN, "Assign your status LED output");
constexpr bool isCourseMotorPin(uint8_t pin) {
  return pin == 4 || pin == 8 || pin == 22 || pin == 23;
}
static_assert(isCourseMotorPin(MOTOR1_CMD_PIN) && isCourseMotorPin(MOTOR2_CMD_PIN) &&
              isCourseMotorPin(MOTOR3_CMD_PIN) && isCourseMotorPin(MOTOR4_CMD_PIN) &&
              MOTOR1_CMD_PIN != MOTOR2_CMD_PIN && MOTOR1_CMD_PIN != MOTOR3_CMD_PIN &&
              MOTOR1_CMD_PIN != MOTOR4_CMD_PIN && MOTOR2_CMD_PIN != MOTOR3_CMD_PIN &&
              MOTOR2_CMD_PIN != MOTOR4_CMD_PIN && MOTOR3_CMD_PIN != MOTOR4_CMD_PIN,
              "Use D4, D8, D22 and D23 once each; motor order is your choice");

constexpr uint8_t RC_PWM_THROTTLE_DBG_PIN = UNASSIGNED_PIN;
constexpr uint8_t RC_PWM_YAW_DBG_PIN = UNASSIGNED_PIN;
constexpr uint8_t RC_PWM_PITCH_DBG_PIN = UNASSIGNED_PIN;
constexpr uint8_t RC_PWM_ROLL_DBG_PIN = UNASSIGNED_PIN;
#if ENABLE_DEBUG_PWM_OUTPUTS
static_assert(RC_PWM_THROTTLE_DBG_PIN != UNASSIGNED_PIN && RC_PWM_YAW_DBG_PIN != UNASSIGNED_PIN &&
              RC_PWM_PITCH_DBG_PIN != UNASSIGNED_PIN && RC_PWM_ROLL_DBG_PIN != UNASSIGNED_PIN,
              "Assign optional scope output pins only if you include them");
#endif

constexpr uint8_t I2C_SDA_PIN = UNASSIGNED_PIN;
constexpr uint8_t I2C_SCL_PIN = UNASSIGNED_PIN;
static_assert(I2C_SDA_PIN != UNASSIGNED_PIN && I2C_SCL_PIN != UNASSIGNED_PIN,
              "Choose a compatible I2C pin pair; update Wire instance too if needed");

constexpr uint8_t VBAT_SENSE_PIN = UNASSIGNED_PIN;
static_assert(VBAT_SENSE_PIN != UNASSIGNED_PIN, "Choose your battery-sense ADC input");

// -----------------------------
// ADC and battery monitor
// -----------------------------
constexpr uint8_t ADC_BITS = 12;
constexpr uint16_t ADC_MAX_VALUE = (1u << ADC_BITS) - 1u;
constexpr float ADC_REFERENCE_VOLTS = 3.3f;  // Teensy 4.0 analog input scale when powered normally.

// Fill these from your own selected divider; zero is not a usable value.
constexpr float R_BAT_TOP_OHMS = 0.0f;
constexpr float R_BAT_BOTTOM_OHMS = 0.0f;
static_assert(R_BAT_TOP_OHMS > 0.0f && R_BAT_BOTTOM_OHMS > 0.0f,
              "Set the actual battery-divider resistor values from your design");
constexpr float BATTERY_DIVIDER_RATIO = (R_BAT_TOP_OHMS + R_BAT_BOTTOM_OHMS) / R_BAT_BOTTOM_OHMS;

// 3S LiPo thresholds for the Tattu 11.1 V pack. These are telemetry warnings;
// automatic motor cutoff remains disabled so a warning cannot stop the drone.
// Cell thresholds copied from the Betaflight Power & Battery tab: warning at
// 3.5 V/cell and minimum at 3.3 V/cell, times three cells.
constexpr float BATTERY_LOW_VOLTS = 10.5f;       // 3.5 V/cell, Betaflight warning
constexpr float BATTERY_CRITICAL_VOLTS = 9.9f;   // 3.3 V/cell, Betaflight minimum
constexpr bool DISABLE_MOTORS_ON_CRITICAL_BATTERY = false;
constexpr uint16_t BATTERY_UPDATE_INTERVAL_MS = 200;

// -----------------------------
// Radio configuration
// -----------------------------
constexpr bool RF24_AUTO_ACK_ENABLED = true;
constexpr uint8_t RADIO_CHANNEL = 76;
constexpr uint8_t RF24_RETRY_DELAY = 3;
constexpr uint8_t RF24_RETRY_COUNT = 5;

// Must match the transmitter exactly. Address width is 5, so the null terminator is not transmitted.
const uint8_t RADIO_ADDRESS[6] = "D426T";

// If changed here, change the transmitter to match: RF24_2MBPS, RF24_1MBPS, or RF24_250KBPS.
constexpr rf24_datarate_e RADIO_DATA_RATE = RF24_2MBPS;

// Receiver PA level mainly affects auto-ack replies. Keep low for bench work unless range testing.
constexpr rf24_pa_dbm_e RADIO_PA_LEVEL = RF24_PA_LOW;

constexpr uint32_t FAILSAFE_TIMEOUT_US = 150000UL;
constexpr uint8_t TELEMETRY_PROTOCOL_VERSION = 1;

// -----------------------------
// Shared nRF packet format
// -----------------------------
constexpr int16_t COMMAND_MIN = -1000;
constexpr int16_t COMMAND_MAX = 1000;
constexpr int16_t THROTTLE_MIN_US = 1000;
constexpr int16_t THROTTLE_MAX_US = 2000;
constexpr int16_t THROTTLE_NEUTRAL_US = 1500;

enum ThrottleMode : uint8_t {
  THROTTLE_HOLD = 0,
  THROTTLE_UP = 1,
  THROTTLE_DOWN = 2
};

struct __attribute__((packed)) ControlPacket {
  uint16_t sequenceNumber;
  int16_t rollCommand;
  int16_t pitchCommand;
  int16_t yawCommand;
  int16_t throttleCommand;
  uint8_t buttons;
  uint8_t flags;
  uint16_t checksum;
};

// Byte-identical to the transmitter's definition. The transmitter prints $T
// rows straight from these fields, so the setpoints have to travel with the
// measurements: without them the tuner would have a response and nothing to
// compare it against, and step-response analysis needs both.
struct __attribute__((packed)) TelemetryPacket {
  uint8_t protocolVersion;
  uint8_t flags;
  uint16_t telemetrySequence;
  int16_t rollCentideg;
  int16_t pitchCentideg;
  int16_t gyroXDeciDps;
  int16_t gyroYDeciDps;
  int16_t gyroZDeciDps;
  int16_t rateSetpointDeciDps[3];
  int16_t angleSetpointCentideg[2];
  uint8_t motorQuarterUs[4];
  uint16_t batteryMillivolts;
  uint16_t checksum;
};

// Dedicated 25 Hz attitude payload for the wireless 3D viewer. Keeping this
// separate from TelemetryPacket preserves every established tuner field while
// allowing yaw and acceleration to cross the 32-byte nRF24 link.
constexpr uint8_t IMU_TELEMETRY_MAGIC = 0x49;
constexpr uint8_t IMU_TELEMETRY_PROTOCOL_VERSION = 1;
enum ImuTelemetryFlags : uint16_t {
  IMU_TELEMETRY_IMU_OK = 1u << 0,
  IMU_TELEMETRY_TIMER_OK = 1u << 1
};

struct __attribute__((packed)) ImuTelemetryPacket {
  uint8_t magic;
  uint8_t protocolVersion;
  uint16_t sequenceNumber;
  int16_t rollCentideg;
  int16_t pitchCentideg;
  int16_t yawCentideg;
  int16_t gyroXDeciDps;
  int16_t gyroYDeciDps;
  int16_t gyroZDeciDps;
  int16_t accelXMilliG;
  int16_t accelYMilliG;
  int16_t accelZMilliG;
  uint16_t flags;
  uint16_t checksum;
};

enum TelemetryFlags : uint8_t {
  TELEMETRY_RADIO_OK = 1u << 0,
  TELEMETRY_IMU_OK = 1u << 1,
  TELEMETRY_TIMER_OK = 1u << 2,
  TELEMETRY_FAILSAFE_ACTIVE = 1u << 3,
  TELEMETRY_STARTUP_SAFETY_CLEARED = 1u << 4,
  TELEMETRY_BATTERY_LOW = 1u << 5,
  TELEMETRY_BATTERY_CRITICAL = 1u << 6,
  TELEMETRY_ANGLE_MODE = 1u << 7
};

// -----------------------------
// Live PID tuning over the radio
// -----------------------------
constexpr uint8_t PID_UPDATE_MAGIC = 0xA5;
constexpr uint8_t PID_STATUS_MAGIC = 0x5A;

enum PidCommand : uint8_t {
  PID_COMMAND_SET = 1,
  PID_COMMAND_GET = 2,
  PID_COMMAND_RESET = 3
};

enum PidControllerId : uint8_t {
  PID_CONTROLLER_ROLL = 0,
  PID_CONTROLLER_PITCH = 1,
  PID_CONTROLLER_YAW = 2,
  PID_CONTROLLER_ROLL_ANGLE = 3,
  PID_CONTROLLER_PITCH_ANGLE = 4,
  // ID 5 remains reserved for older receivers' motor trim. Level trim gets a
  // new ID so the shared transmitter can remain backward compatible.
  PID_CONTROLLER_LEVEL_TRIM = 6
};

enum PidStatusCode : uint8_t {
  PID_STATUS_APPLIED = 0,
  PID_STATUS_CURRENT = 1,
  PID_STATUS_RESET_TO_DEFAULT = 2,
  PID_STATUS_INVALID_PACKET = 3,
  PID_STATUS_INVALID_VALUES = 4,
  PID_STATUS_UNKNOWN_CONTROLLER = 5,
  PID_STATUS_UNKNOWN_COMMAND = 6
};

struct __attribute__((packed)) PidUpdatePacket {
  uint8_t magic;
  uint8_t command;
  uint8_t controllerId;
  uint8_t reserved;
  uint16_t sequenceNumber;
  float kp;
  float ki;
  float kd;
  uint16_t checksum;
};

struct __attribute__((packed)) PidStatusPacket {
  uint8_t magic;
  uint8_t status;
  uint8_t controllerId;
  uint8_t command;
  uint16_t sequenceNumber;
  float kp;
  float ki;
  float kd;
  uint16_t checksum;
};

// Passive onboard rate-loop trace. The transmitter already recognizes this
// unique 24-byte downlink payload and prints it as PID_TRACE for TunerV2.
// These are the actual controller contributions used by the flight loop, not
// browser-side estimates. No trace field is ever fed back into flight control.
constexpr uint8_t PID_TRACE_MAGIC = 0x3C;

struct __attribute__((packed)) PidTracePacket {
  uint8_t magic;
  uint8_t axisId;
  uint8_t flags;
  uint8_t reserved;
  uint16_t captureId;
  uint16_t sampleIndex;
  uint16_t sampleCount;
  int16_t relativeMs;
  int16_t setpointDeciDps;
  int16_t measuredDeciDps;
  int16_t pTermCentiUs;
  int16_t iTermCentiUs;
  int16_t dTermCentiUs;
  uint16_t checksum;
};

enum PidTraceFlags : uint8_t {
  PID_TRACE_FIRST_SAMPLE = 1u << 0,
  PID_TRACE_LAST_SAMPLE = 1u << 1,
  PID_TRACE_MOTOR_SATURATED = 1u << 2,
  PID_TRACE_DISTURBANCE = 1u << 3,
  PID_TRACE_TERM_CLIPPED = 1u << 4
};

// -----------------------------
// Identity and descriptor transfer
// -----------------------------
// The tuner caches the descriptor by content hash. Name+version alone would not
// do: editing the descriptor without bumping the version would silently serve a
// stale interface.
constexpr uint8_t PID_IDENTITY_MAGIC = 0x7E;

struct __attribute__((packed)) PidIdentityPacket {
  uint8_t magic;
  uint8_t nameLen;
  uint32_t descriptorHash;
  char name[14];
  char version[6];
  uint16_t checksum;
};

constexpr uint8_t PID_DESCRIPTOR_REQUEST_MAGIC = 0xD1;
constexpr uint8_t PID_DESCRIPTOR_CHUNK_MAGIC = 0xD2;
constexpr uint8_t PID_DESCRIPTOR_DATA_BYTES = 19;
constexpr uint16_t PID_DESCRIPTOR_MAX_CHUNKS = 256;

struct __attribute__((packed)) PidDescriptorRequestPacket {
  uint8_t magic;
  uint8_t reserved;
  uint16_t index;
  uint32_t descriptorHash;
  uint16_t checksum;
};

struct __attribute__((packed)) PidDescriptorChunkPacket {
  uint8_t magic;
  uint16_t index;
  uint16_t total;
  uint8_t length;
  uint32_t descriptorHash;
  char data[PID_DESCRIPTOR_DATA_BYTES];
  uint16_t checksum;
};

static_assert(sizeof(ControlPacket) <= 32, "ControlPacket must fit nRF24L01 payload limit");
static_assert(sizeof(ControlPacket) == 14, "Receiver packet must match transmitter packet size");
static_assert(sizeof(TelemetryPacket) == 32, "TelemetryPacket must be exactly one nRF24L01 payload");
static_assert(sizeof(ImuTelemetryPacket) == 26, "IMU telemetry packet must be 26 bytes");
static_assert(sizeof(PidUpdatePacket) == 20, "PID update packet must be 20 bytes");
static_assert(sizeof(PidStatusPacket) == 20, "PID status packet must be 20 bytes");
static_assert(sizeof(PidTracePacket) == 24, "PID trace packet must be 24 bytes");
static_assert(sizeof(PidIdentityPacket) == 28, "PID identity packet must be 28 bytes");
static_assert(sizeof(PidDescriptorRequestPacket) == 10, "Descriptor request packet must be 10 bytes");
static_assert(sizeof(PidDescriptorChunkPacket) == 31, "Descriptor chunk packet must be 31 bytes");

// Both sides tell packet types apart purely by payload size, so every size on
// the link must stay unique. Adding a field to any of these without checking
// here is how two packet types silently become each other.
static_assert(sizeof(PidUpdatePacket) != sizeof(ControlPacket) &&
              sizeof(PidUpdatePacket) != sizeof(TelemetryPacket) &&
              sizeof(PidTracePacket) != sizeof(ControlPacket) &&
              sizeof(PidTracePacket) != sizeof(PidUpdatePacket) &&
              sizeof(PidTracePacket) != sizeof(PidStatusPacket) &&
              sizeof(PidTracePacket) != sizeof(PidIdentityPacket) &&
              sizeof(PidTracePacket) != sizeof(PidDescriptorRequestPacket) &&
              sizeof(PidTracePacket) != sizeof(PidDescriptorChunkPacket) &&
              sizeof(PidTracePacket) != sizeof(TelemetryPacket) &&
              sizeof(PidIdentityPacket) != sizeof(ControlPacket) &&
              sizeof(PidIdentityPacket) != sizeof(PidUpdatePacket) &&
              sizeof(PidIdentityPacket) != sizeof(TelemetryPacket) &&
              sizeof(PidDescriptorRequestPacket) != sizeof(ControlPacket) &&
              sizeof(PidDescriptorRequestPacket) != sizeof(PidUpdatePacket) &&
              sizeof(PidDescriptorRequestPacket) != sizeof(PidIdentityPacket) &&
              sizeof(PidDescriptorRequestPacket) != sizeof(TelemetryPacket) &&
              sizeof(PidDescriptorChunkPacket) != sizeof(ControlPacket) &&
              sizeof(PidDescriptorChunkPacket) != sizeof(PidUpdatePacket) &&
              sizeof(PidDescriptorChunkPacket) != sizeof(PidIdentityPacket) &&
              sizeof(PidDescriptorChunkPacket) != sizeof(PidDescriptorRequestPacket) &&
              sizeof(PidDescriptorChunkPacket) != sizeof(TelemetryPacket) &&
              sizeof(ImuTelemetryPacket) != sizeof(ControlPacket) &&
              sizeof(ImuTelemetryPacket) != sizeof(PidUpdatePacket) &&
              sizeof(ImuTelemetryPacket) != sizeof(PidTracePacket) &&
              sizeof(ImuTelemetryPacket) != sizeof(PidIdentityPacket) &&
              sizeof(ImuTelemetryPacket) != sizeof(PidDescriptorRequestPacket) &&
              sizeof(ImuTelemetryPacket) != sizeof(PidDescriptorChunkPacket) &&
              sizeof(ImuTelemetryPacket) != sizeof(TelemetryPacket),
              "Radio packet sizes must stay unique so size-based dispatch is unambiguous");

// -----------------------------
// MPU6050 register-level setup
// -----------------------------
constexpr uint8_t MPU6050_I2C_ADDR_LOW = 0x68;
constexpr uint8_t MPU6050_I2C_ADDR_HIGH = 0x69;
uint8_t mpu6050I2cAddress = MPU6050_I2C_ADDR_LOW;
constexpr uint8_t MPU6050_REG_SMPLRT_DIV = 0x19;
constexpr uint8_t MPU6050_REG_CONFIG = 0x1A;
constexpr uint8_t MPU6050_REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t MPU6050_REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t MPU6050_REG_INT_PIN_CFG = 0x37;
constexpr uint8_t MPU6050_REG_INT_ENABLE = 0x38;
constexpr uint8_t MPU6050_REG_INT_STATUS = 0x3A;
constexpr uint8_t MPU6050_REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t MPU6050_REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t MPU6050_REG_WHO_AM_I = 0x75;
constexpr uint8_t MPU6050_CMD_DEVICE_RESET = 0x80;
constexpr uint8_t MPU6050_WAKE_PLL_XGYRO = 0x01;
constexpr uint8_t MPU6050_FS_SEL_MASK = 0x18;
constexpr uint8_t MPU6050_WHO_AM_I_VALUES[] = {
    0x68, 0x69, 0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x98};
// dRehm profile: DLPF bypass gives the MPU gyro's 8 kHz internal path and
// accel's 1 kHz path. The control task reads the most recent register sample
// at 2 kHz; MPU6050 register reads are not a claim of fresh 2 kHz accel data.
constexpr uint8_t MPU6050_DLPF_OFF = 0x00;
constexpr uint8_t MPU6050_SAMPLE_RATE_DIV = 0;
constexpr uint16_t MPU6050_GYRO_PATH_HZ = 8000;
constexpr uint16_t MPU6050_ACCEL_PATH_HZ = 1000;
static_assert(MPU6050_SAMPLE_RATE_DIV == 0, "dRehm profile must not divide gyro path");
constexpr uint8_t MPU6050_ACCEL_RANGE_4G = 0x08;
constexpr uint8_t MPU6050_GYRO_RANGE_2000DPS = 0x18;
constexpr float IMU_ACC_LSB_PER_G = 8192.0f;
constexpr float IMU_GYRO_LSB_PER_DPS = 16.4f;
// With the requested upright mounting, a level stationary sensor reads +1 g on Z.
constexpr float IMU_LEVEL_SENSOR_Z_G = 1.0f;

/* ---------------------------------------------------------------------------
   MPU6050 SAMPLE RATE AND FILTERING

   This EXPERIMENTAL branch applies the dRehm-style raw MPU profile: CONFIG=0
   (DLPF off), gyro internal path 8 kHz and accel internal path 1 kHz. Its
   controller runs every 500 us. Wire is requested at 1 MHz, which exceeds the
   baseline I2C specification for MPU6050 modules and must be proven on the
   actual wiring with a scope/log before flight.

   The IMU sampling clock and the control timer are independent. Do not assume
   a fixed sample phase. This is a compile/static-checked experiment, not a
   timing or flight-validation result.
   --------------------------------------------------------------------------- */

/* ---------------------------------------------------------------------------
   MEASUREMENT RANGES

   Accel stays at +/-4 g, matching the MPU6050 build.

   Gyro is +/-2000 deg/s. It used to be +/-500 to match the MPU build, and that
   was wrong on both builds: a gyro that saturates stops being a feedback sensor
   at exactly the moment you need it. Past the limit the rate loop sees a
   clamped value, concludes the rotation is slower than it really is,
   under-corrects, and the rotation keeps building. That failure is invisible at
   rest, invisible on a slow hand tilt, and invisible on the seesaw rig, which
   never gets near the limit - it only appears once something is already
   rotating fast.

   +/-500 deg/s is also below this firmware's own reference tune: the converted
   Betaflight rate curve reaches 666.7 deg/s at full stick, which the sensor
   could not measure. The selected MPU6050 range instead measures up to
   +/-2000 deg/s.

   The cost is resolution: 16.4 LSB per deg/s instead of 65.6, so one count is
   0.061 deg/s rather than 0.015. The measured noise floor on this part is
   0.4-0.6 deg/s, an order of magnitude above either, so quantisation is not the
   limiting factor and nothing is lost.

   Read-back consequence, worth knowing: the MPU6050 range is explicitly
   default, so a silently-failed GYR_RANGE write now leaves the part on exactly
   the range we wanted. The read-back below can no longer distinguish "the write
   stuck" from "the write did nothing" - but it no longer needs to, because both
   outcomes are correct. The accel read-back still discriminates, since its
   default is +/-2 g and we ask for +/-4 g.
   --------------------------------------------------------------------------- */

/* ---------------------------------------------------------------------------
   SENSOR ORIENTATION ON THE FINAL PCB

   Sensor +X points left, +Y points back, and +Z points up. Body +X remains
   front and body +Y remains left, so body=(-sensorY,sensorX,sensorZ).
   Apply this transform to both acceleration and gyro after subtracting their
   sensor-frame startup biases.
   Measured body +Z yaw is nose-left positive and nose-right negative. The
   established transmitter stick mapping is preserved. Positive yaw mixer
   output increases the CW pair M1/M4 and reduces the CCW pair M2/M3, matching
   the proven airframe's physical torque convention.
   --------------------------------------------------------------------------- */

// Bias capture runs for a fixed wall-clock window rather than a fixed sample
// count, so the averaging time does not change if the I2C reads speed up or
// slow down. The LINK LED fast-flashes for the whole window: hold the airframe
// still and level until it stops flashing.
constexpr uint32_t IMU_CALIBRATION_DURATION_MS = 10000;
// Bias estimation does not need every flight-rate sample. The 2 ms delay caps
// calibration sampling near 500 Hz and leaves time for I2C and ESC stop frames.
constexpr uint16_t IMU_CALIBRATION_SAMPLE_PERIOD_MS = 2;
constexpr uint16_t IMU_CALIBRATION_LED_PERIOD_MS = 100;   // 5 Hz flash while calibrating
constexpr uint32_t IMU_CALIBRATION_MIN_SAMPLES = 500;
// Motion is checked on a rolling mean after three-sample median filtering.
constexpr uint8_t IMU_CALIBRATION_MOTION_WINDOW_SAMPLES = 25;
constexpr float IMU_CALIBRATION_MOTION_LIMIT_DPS = 8.0f;

// Fixed dRehm-style per-update blends, intentionally not tau-derived.
constexpr float DREHM_BGYRO = 0.10f;
constexpr float DREHM_BACCEL = 0.14f;
constexpr float DREHM_MADGWICK_BETA = 0.04f;
constexpr float RAD_TO_DEG_LOCAL = 57.2957795131f;

// -----------------------------
// Flight-control timing
// -----------------------------
// Experimental dRehm-controlANGLE2 branch: these gains require flight retuning.
constexpr uint16_t CONTROL_LOOP_HZ = 2000;
constexpr uint32_t CONTROL_LOOP_PERIOD_US = 1000000UL / CONTROL_LOOP_HZ;
constexpr float CONTROL_LOOP_PERIOD_S = static_cast<float>(CONTROL_LOOP_PERIOD_US) / 1000000.0f;
static_assert(CONTROL_LOOP_PERIOD_US == 500,
              "The dRehm branch must schedule a 500 us control period");
constexpr uint16_t DEBUG_PWM_PERIOD_MS = 20;
constexpr uint16_t LED_UPDATE_INTERVAL_MS = 100;
constexpr uint16_t SERIAL_DEBUG_INTERVAL_MS = 200;

// -----------------------------
// PID and motor configuration
// -----------------------------
// ---------------------------------------------------------------------------
// Stick-to-rate curve, from the Betaflight rateprofile flown on this airframe:
// RC Rate 1.00, Super Rate 0.70, RC Expo 0.00 on all three axes.
//
// Betaflight's curve, which this reproduces exactly:
//   expo:  s = s*|s|^3*expo + s*(1 - expo)
//   base:  rate = 200 * rcRate * s
//   super: rate *= 1 / (1 - |s| * superRate)
//
// At full stick that is 200 / (1 - 0.70) = 666.7 deg/s, matching the 667 the
// configurator reports. The point of the curve is that it is NOT linear: half
// stick gives about 154 deg/s rather than 333, so the centre is soft for
// holding a hover while the ends stay fast.
// ---------------------------------------------------------------------------
constexpr float BF_RC_RATE = 1.00f;
constexpr float BF_SUPER_RATE = 0.70f;
constexpr float BF_RC_EXPO = 0.00f;
constexpr float BF_SUPER_RATE_MAX = 0.99f;  // guards the 1/(1-x) singularity
// Applied only to roll and pitch before their angle or rate setpoint generation.
// y = (1 - expo) * x + expo * x^3 retains zero and full-stick authority.
constexpr float ROLL_PITCH_CUBIC_EXPO = 0.50f;

// Rate ceilings. The curve above is SCALED to these rather than clipped by
// them, so full stick always lands exactly on the ceiling and the soft centre
// is preserved at any setting. Set all three to 667 to fly the Betaflight
// rateprofile verbatim - at 667 the scale factor is 1.0 and the curve is
// untouched.
//
// Held at the original firmware's values for now. These are compile-time, so a
// number that turns out to be too much cannot be walked back from the tuner
// mid-flight the way a PID gain can. Raise them once it hovers cleanly.
//
// Note which of these actually reaches the sticks: in ANGLE mode roll and pitch
// come from the outer loop, already limited by ANGLE_OUTPUT_RATE_LIMIT_DPS, so
// the two below only act as a backstop. Yaw is rate-controlled in BOTH modes,
// so MAX_YAW_RATE_DPS is felt directly whichever mode you are in.
constexpr float MAX_ROLL_RATE_DPS = 180.0f;   // Betaflight rateprofile: 667
constexpr float MAX_PITCH_RATE_DPS = 180.0f;  // Betaflight rateprofile: 667
constexpr float MAX_YAW_RATE_DPS = 160.0f;    // Betaflight rateprofile: 667

// TPA, from Betaflight: 0.65 above a 1250 us breakpoint. Above the breakpoint
// P and D are scaled down toward 35% at full throttle, because a quad that is
// stable at hover gains authority as the props speed up and will oscillate on
// the same gains. I is deliberately left alone, exactly as Betaflight does it.
constexpr float TPA_RATE = 0.65f;
constexpr int16_t TPA_BREAKPOINT_US = 1250;

// Throttle Limit SCALE 75%. Caps commanded throttle at 75% of its range, which
// is the first-flight limiter already in use on the Betaflight build. Set to
// 1.0f to remove it.
constexpr float THROTTLE_LIMIT_SCALE = 0.75f;

// ANGLE mode: the sticks command a lean angle and releasing them returns the
// airframe to level. ACRO mode: the sticks command a rotation rate and the
// airframe holds whatever attitude it was left at. Yaw is always rate-controlled
// in both modes, because there is no heading reference to level against.
enum FlightMode : uint8_t {
  FLIGHT_MODE_ACRO = 0,
  FLIGHT_MODE_ANGLE = 1
};

constexpr FlightMode FLIGHT_MODE = FLIGHT_MODE_ANGLE;
bool angleModeEnabled = (FLIGHT_MODE == FLIGHT_MODE_ANGLE);

// Full stick deflection in angle mode. This is THE number that decides how
// aggressive angle mode feels, because in angle mode the stick commands a lean
// angle directly.
//
// Betaflight flew this airframe at 55. Held at 25 for now: it is compile-time,
// so unlike a PID gain it cannot be reduced from the tuner if the first hover
// turns out livelier than expected. Raise it toward 55 once the aircraft is
// trusted - nothing else depends on this number.
constexpr float MAX_LEVEL_ANGLE_DEG = 25.0f;  // Betaflight Angle Limit: 55

struct PidGains {
  float kp;
  float ki;
  float kd;
};

// Instrumentation observes every fourth 2 kHz control tick: actual 500 Hz.
constexpr uint16_t TRACE_CONTROL_TICK_DIVIDER = 4;
constexpr uint16_t TRACE_RATE_HZ = CONTROL_LOOP_HZ / TRACE_CONTROL_TICK_DIVIDER;
constexpr uint16_t TRACE_PRE_SAMPLES = 100;   // 200 ms at 500 Hz
// Half-open [-200 ms, +500 ms): the trigger sample is the first post sample.
constexpr uint16_t TRACE_POST_SAMPLES = 250;
constexpr uint16_t TRACE_TOTAL_SAMPLES = TRACE_PRE_SAMPLES + TRACE_POST_SAMPLES;
constexpr uint16_t TRACE_RING_SAMPLES = TRACE_TOTAL_SAMPLES + 8;
constexpr float TRACE_TRIGGER_DELTA_DPS = 40.0f;
constexpr uint16_t TRACE_TRIGGER_LOOKBACK = 10;  // 20 ms at 500 Hz
constexpr uint16_t TRACE_REARM_SAMPLES = 500;    // 1 s after a drain ends
constexpr float TRACE_DISTURBANCE_ERROR_DPS = 60.0f;
constexpr float TRACE_DISTURBANCE_STEADY_DPS = 15.0f;
// Three trace ACKs, then one ordinary telemetry ACK.
constexpr uint8_t TRACE_TELEMETRY_INTERLEAVE = 4;
constexpr uint16_t TRACE_DRAIN_MAX_FAILURES = 200;

struct TraceSample {
  uint32_t timestampUs;
  int16_t setpointDeciDps[3];
  int16_t measuredDeciDps[3];
  int16_t pTermCentiUs[3];
  int16_t iTermCentiUs[3];
  int16_t dTermCentiUs[3];
  uint8_t motorSaturated;
  uint8_t termClippedMask;
};

enum TraceState : uint8_t {
  TRACE_IDLE = 0,
  TRACE_CAPTURING = 1,
  TRACE_DRAINING = 2
};

static_assert(CONTROL_LOOP_HZ % TRACE_CONTROL_TICK_DIVIDER == 0,
              "Trace divider must evenly divide the control loop");
static_assert(TRACE_RATE_HZ == 500, "TunerV2 trace must sample at 500 Hz");
static_assert(TRACE_TOTAL_SAMPLES == 350, "Trace must contain 200 ms pre + 500 ms post");
static_assert(TRACE_POST_SAMPLES > 0, "Trace must include its trigger sample");
static_assert(sizeof(TraceSample) == 36, "Unexpected trace sample size/RAM growth");
static_assert(sizeof(TraceSample) * TRACE_RING_SAMPLES <= 13u * 1024u,
              "Trace ring must remain below its 13 KiB RAM budget");

// ---------------------------------------------------------------------------
// Historical starting gains were converted from a Betaflight 4.1.1 tune flown
// on this airframe (OMNIBUSF4SD, profile 3, OneShot125).
//
// The conversion is exact rather than a guess. Betaflight forms its P term as
// PTERM_SCALE * P * errorRate with PTERM_SCALE = 0.032029, then divides pidSum
// by PID_MIXER_SCALING = 1000 to get a fraction of the motor range. Over a
// 1000 us motor range that fraction is numerically microseconds again, so
// Betaflight's pidSum and this firmware's PID output are the same unit:
//
//     kp [us per deg/s] = 0.032029 * P_betaflight
//     kd [us per deg/s^2] = 0.000529 * D_betaflight
//
// The defaults below supersede those calculated starting values: they record
// the operator's later live-tuning result rather than a direct Betaflight port.
// ---------------------------------------------------------------------------
// dRehm-tailored power-on/Reset defaults. Further flight validation remains
// the operator's responsibility after any airframe or prop change.
constexpr PidGains DEFAULT_ROLL_RATE_PID = {0.200f, 0.540f, 0.00020f};
constexpr PidGains DEFAULT_PITCH_RATE_PID = {0.320f, 0.210f, 0.00020f};
constexpr PidGains DEFAULT_YAW_RATE_PID = {0.115f, 0.300f, 0.00015f};

// Outer angle loop, in deg/s of commanded rotation per degree of angle error.
// Betaflight's angle strength 50 becomes pid[PID_LEVEL].P / 10 = 5.0 deg/s per
// deg, which replaces this sketch's previous fixed 6.7 proportional-only law.
// Making it a full PID is what lets the tuner treat it as a controller at all.
constexpr PidGains DEFAULT_ROLL_ANGLE_PID = {0.166667f, 0.023333f, 0.0f};
constexpr PidGains DEFAULT_PITCH_ANGLE_PID = {0.166667f, 0.026667f, 0.0f};

/* Hover-level trim shifts the Angle-mode target instead of biasing motor
   outputs. It corrects small mounting, CG, and thrust-vector offsets while
   leaving the tested motor mixer and closed-loop corrections intact. */
constexpr float LEVEL_TRIM_LIMIT_DEG = 5.0f;
// Avoid an abrupt attitude command if a trim is changed while airborne. At
// 1 deg/s, the normal 0.1-0.25 degree tuning steps settle quickly, while even a
// mistaken full-scale entry takes five seconds to reach the controller.
constexpr float LEVEL_TRIM_SLEW_RATE_DEG_PER_S = 1.0f;
// Reuse the packet payload: kp=roll degrees, ki=pitch degrees, kd=unused.
constexpr PidGains DEFAULT_LEVEL_TRIM = {0.0f, 1.3f, 0.0f};

// Live copies. Everything above is the reset target, never the working value.
PidGains rollRatePid = DEFAULT_ROLL_RATE_PID;
PidGains pitchRatePid = DEFAULT_PITCH_RATE_PID;
PidGains yawRatePid = DEFAULT_YAW_RATE_PID;
PidGains rollAnglePid = DEFAULT_ROLL_ANGLE_PID;
PidGains pitchAnglePid = DEFAULT_PITCH_ANGLE_PID;

PidGains levelTrimDeg = DEFAULT_LEVEL_TRIM;
float activeLevelTrimRollDeg = 0.0f;
float activeLevelTrimPitchDeg = 0.0f;

// Bounds must match the descriptor's declared min/max, because the tuner trusts
// the descriptor and the firmware must not.
constexpr float PID_KP_MAX = 2.0f;
constexpr float PID_KI_MAX = 5.0f;
constexpr float PID_KD_MAX = 0.01f;
constexpr float ANGLE_KP_MAX = 1.0f;
constexpr float ANGLE_KI_MAX = 0.5f;
constexpr float ANGLE_KD_MAX = 0.2f;
// dRehm stores a raw accumulated error, unlike the prior controller's
// output-unit integrator. Keep this declaration before live gain handling.
constexpr float DREHM_INTEGRATOR_RAW_LIMIT = 25.0f;

// The outer loop outputs a rate, so its limits are deg/s, not microseconds.
constexpr float ANGLE_INTEGRATOR_LIMIT_DPS = 60.0f;
constexpr float ANGLE_OUTPUT_RATE_LIMIT_DPS = 180.0f;
constexpr float ANGLE_D_TERM_LIMIT_DPS = 40.0f;

// Inner rate loop. Outputs are motor correction microseconds, error is deg/s.
//
// Kp comes from the seesaw lab sketch: its combined law was
//   output = 8.0*angleError - 1.2*gyroRate
// and the gyroRate coefficient is exactly the rate-loop Kp once the law is split
// into a cascade, so 1.2 is a measured number, not a guess.
//
// Ki is a starting point, not a derived value. The previous 0.25 was inert:
// with a 50 deg/s error it needs roughly 12 seconds to reach the integrator
// clamp, so the loop was effectively P-only. The dt-aware update keeps that
// time behavior across loop rates; at 3.0 the same error reaches clamp in about a second.
//
// Kd starts at zero on purpose. The damping the lab sketch got from its -Kd*gyro
// term now lives in the rate-loop Kp, so adding rate D on top would double it.
// Add D only after P and I are tuned, and only if you see residual oscillation.
//
// Tune all of this on a secured rig with props off before you fly it.
constexpr float PID_INTEGRATOR_LIMIT_US = 150.0f;
constexpr float PID_OUTPUT_LIMIT_US = 250.0f;
constexpr float PID_D_TERM_LIMIT_US = 30.0f;
constexpr float PID_D_FILTER_TAU_S = 0.012f;

constexpr int16_t MOTOR_MIN_US = 1000;
// 1070, matching the Betaflight min_throttle that these ESCs and motors were
// actually flown at. The previous 1050 is close enough to the ESC's arming
// threshold that a motor can fail to start turning while its three neighbours
// do, which the mix then reads as an attitude error it cannot correct.
constexpr int16_t MOTOR_IDLE_US = 1070;
constexpr int16_t MOTOR_MAX_US = 2000;
constexpr int16_t THROTTLE_LOW_CUTOFF_US = 1030;
constexpr int16_t STARTUP_SAFE_THROTTLE_US = 1050;

// Refuse to arm while the attitude estimate is far from level. A large starting
// error makes the level loop demand a correction bigger than the motors can
// deliver the instant throttle comes up, which is how you get two motors at
// full and two at idle. Checked only at arm time, never in flight.
constexpr float ARM_MAX_TILT_DEG = 15.0f;

// Bit 0 of the control packet's button byte is the transmitter's joystick push
// switch. It was already being transmitted and already being stored here, but
// nothing ever read it; it is now the arm toggle. The left-stick calibration
// request uses the spare bit 1, preserving every packet size.
constexpr uint8_t CONTROL_BUTTON_ARM_MASK = 0x01;
constexpr uint8_t CONTROL_BUTTON_CAL_MASK = 0x02;
// Bit 7 is emitted only by the tuner-aware transmitter bridge. It preserves
// the ControlPacket layout while making OUT,STOP an explicit receiver event.
constexpr uint8_t CONTROL_BUTTON_REMOTE_STOP_MASK = 0x80;

// The switch is read straight off a pin on the transmitter with no debouncing,
// so one physical press arrives as several pressed/released edges. Without a
// lockout each of those would toggle the arm state and leave it anyone's guess.
constexpr uint32_t ARM_TOGGLE_LOCKOUT_MS = 300;

constexpr int16_t FAILSAFE_THROTTLE_US = 1000;
constexpr int16_t FAILSAFE_ROLL = 0;
constexpr int16_t FAILSAFE_PITCH = 0;
constexpr int16_t FAILSAFE_YAW = 0;

enum MotorOutputProtocol : uint8_t {
  MOTOR_PROTOCOL_STANDARD_PWM = 0,
  MOTOR_PROTOCOL_ONESHOT125 = 1,
  MOTOR_PROTOCOL_DSHOT300 = 2,
  MOTOR_PROTOCOL_DSHOT600 = 3
};

// ReceiverProtocolConfig.h validates that exactly one branch is enabled.
#if ESC_PROTOCOL_PWM
constexpr MotorOutputProtocol MOTOR_PROTOCOL = MOTOR_PROTOCOL_STANDARD_PWM;
#elif ESC_PROTOCOL_ONESHOT125
constexpr MotorOutputProtocol MOTOR_PROTOCOL = MOTOR_PROTOCOL_ONESHOT125;
#elif ESC_PROTOCOL_DSHOT300
constexpr MotorOutputProtocol MOTOR_PROTOCOL = MOTOR_PROTOCOL_DSHOT300;
#elif ESC_PROTOCOL_DSHOT600
constexpr MotorOutputProtocol MOTOR_PROTOCOL = MOTOR_PROTOCOL_DSHOT600;
#endif

// The shared standard-PWM backend remains below for code reuse, but the 2 kHz
// compile-time guard above deliberately prohibits selecting it.
constexpr uint16_t MOTOR_PWM_FREQUENCY_HZ = CONTROL_LOOP_HZ;
constexpr uint8_t MOTOR_PWM_RESOLUTION_BITS = 12;
constexpr int32_t MOTOR_PWM_MAX_COUNT = (1L << MOTOR_PWM_RESOLUTION_BITS) - 1L;
constexpr float MOTOR_PWM_FRAME_US = 1000000.0f / static_cast<float>(MOTOR_PWM_FREQUENCY_HZ);

// DShot uses a 16-bit packet: 11-bit command, telemetry request, and 4-bit CRC.
// Values 1..47 are special commands, 48..2047 are throttle, and 0 is stop.
// Packet encoding retains the teensySHOT-compatible unidirectional format.
// The optional GPIO writer below replaces the incomplete DMA backend.
constexpr uint16_t DSHOT_STOP_VALUE = 0;
constexpr uint16_t DSHOT_MIN_THROTTLE = 48;
constexpr uint16_t DSHOT_MAX_THROTTLE = 2047;
constexpr bool DSHOT_REQUEST_TELEMETRY = false;
constexpr bool MOTOR_PROTOCOL_IS_DSHOT =
    MOTOR_PROTOCOL == MOTOR_PROTOCOL_DSHOT300 ||
    MOTOR_PROTOCOL == MOTOR_PROTOCOL_DSHOT600;

// The optional DShot output uses a bounded DWT cycle-counter GPIO writer.
// It does not reserve DMA channels or leave FlexPWM running between packets.
// Validate the waveform on a scope before using this optional output in flight.
#if ESC_PROTOCOL_DSHOT300 || ESC_PROTOCOL_DSHOT600
constexpr uint8_t DSHOT_MOTOR_COUNT = 4;
constexpr uint8_t DSHOT_FRAME_BITS = 16;
constexpr uint32_t DSHOT_SPEED_SCALE = ESC_PROTOCOL_DSHOT300 ? 2UL : 1UL;
constexpr uint32_t DSHOT_BIT_DURATION_NS = 1670UL * DSHOT_SPEED_SCALE;
constexpr uint32_t DSHOT_LONG_PULSE_NS = 1250UL * DSHOT_SPEED_SCALE;
constexpr uint32_t DSHOT_SHORT_PULSE_NS = 625UL * DSHOT_SPEED_SCALE;
constexpr uint32_t DSHOT_LOW_GAP_US = 20;
constexpr uint32_t DSHOT_MAX_WAIT_POLLS = 2048;
constexpr uint32_t DSHOT_MAX_EDGE_LATENESS_NS = 125;
bool dshotOutputHealthy = false;
uint32_t dshotOutputFaultCount = 0;

static_assert(MOTOR1_CMD_PIN == 8 && MOTOR2_CMD_PIN == 4 &&
              MOTOR3_CMD_PIN == 22 && MOTOR4_CMD_PIN == 23,
              "DShot grouped GPIO writes require the V-Final pin map");
static_assert(DSHOT_SHORT_PULSE_NS < DSHOT_LONG_PULSE_NS &&
              DSHOT_LONG_PULSE_NS < DSHOT_BIT_DURATION_NS,
              "DShot pulse timing is invalid");
#endif

// AM32 40 A ESC note:
// Do not connect or parallel ESC 5 V BEC outputs to the PCB 5 V rail by default.

struct MotorMix {
  const char *label;
  int8_t rollSign;
  int8_t pitchSign;
  int8_t yawSign;
};

constexpr MotorMix MOTOR_MIX[4] = {
  {"M1 rear-right",  -1, +1, +1},  // CW
  {"M2 rear-left",   +1, +1, -1},  // CCW
  {"M3 front-right", -1, -1, -1},  // CCW
  {"M4 front-left",  +1, -1, +1}   // CW
};

// -----------------------------
// Runtime state
// -----------------------------
struct ReceiverCommand {
  int16_t roll = FAILSAFE_ROLL;
  int16_t pitch = FAILSAFE_PITCH;
  int16_t yaw = FAILSAFE_YAW;
  int16_t throttle = FAILSAFE_THROTTLE_US;
  uint8_t buttons = 0;
  uint8_t flags = 0;
  uint16_t sequenceNumber = 0;
};

struct ImuState {
  float axG = 0.0f;
  float ayG = 0.0f;
  float azG = 1.0f;
  float gxDps = 0.0f;
  float gyDps = 0.0f;
  float gzDps = 0.0f;
  // rawRollDeg/rawPitchDeg are the fused estimate before the tuner's stored
  // level reference; rollDeg/pitchDeg are what the control loops consume.
  float rawRollDeg = 0.0f;
  float rawPitchDeg = 0.0f;
  float rollDeg = 0.0f;
  float pitchDeg = 0.0f;
  float yawDeg = 0.0f;
  // Madgwick 6DOF quaternion. Body convention remains +X front,+Y left,+Z up.
  float q0 = 1.0f;
  float q1 = 0.0f;
  float q2 = 0.0f;
  float q3 = 0.0f;
};

struct ImuBias {
  float axG = 0.0f;
  float ayG = 0.0f;
  float azG = 0.0f;
  float gxDps = 0.0f;
  float gyDps = 0.0f;
  float gzDps = 0.0f;
};

struct ImuCalibrationCapture {
  float motionWindowDps[3][IMU_CALIBRATION_MOTION_WINDOW_SAMPLES] = {};
  float motionSumDps[3] = {};
  float motionMinDps[3] = {1.0e9f, 1.0e9f, 1.0e9f};
  float motionMaxDps[3] = {-1.0e9f, -1.0e9f, -1.0e9f};
  uint8_t motionWindowIndex = 0;
  uint8_t motionWindowCount = 0;
  float gyroHistoryDps[3][3] = {};
  uint8_t gyroHistoryIndex = 0;
  uint8_t gyroHistoryCount = 0;
  float axSum = 0.0f, aySum = 0.0f, azSum = 0.0f;
  float gxSum = 0.0f, gySum = 0.0f, gzSum = 0.0f;
  float gyroMinDps[3] = {1.0e9f, 1.0e9f, 1.0e9f};
  float gyroMaxDps[3] = {-1.0e9f, -1.0e9f, -1.0e9f};
  uint32_t samples = 0;
  uint32_t startMs = 0;
  uint32_t lastSampleMs = 0;
};

struct PidState {
  float integrator = 0.0f;
  float previousMeasurement = 0.0f;
  float filteredDerivativeTerm = 0.0f;
  float pTerm = 0.0f;
  float iTerm = 0.0f;
  float dTerm = 0.0f;
  bool measurementInitialized = false;
  float previousOutput = 0.0f;
};

RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
IntervalTimer controlTimer;

#if ENABLE_DEBUG_PWM_OUTPUTS
Servo debugThrottlePwm;
Servo debugYawPwm;
Servo debugPitchPwm;
Servo debugRollPwm;
#endif

ReceiverCommand receiverCommand;
ControlPacket latestPacket;
TelemetryPacket telemetryPacket = {};
ImuTelemetryPacket imuTelemetryPacket = {};
ImuState imu;
ImuBias imuBias;
ImuCalibrationCapture imuCalibrationCapture;
PidState rollPidState;
PidState pitchPidState;
PidState yawPidState;
PidState rollAnglePidState;
PidState pitchAnglePidState;
// Never integrates anything; it exists only so level trim can travel through
// the same set/get/reset plumbing as a real controller.
PidState levelTrimUnusedState;

// Setpoints the tuner plots next to each measurement. Held from the last
// control tick so telemetry and the $T stream always report the commanded value
// that produced the response in the same row.
float lastDesiredRollRateDps = 0.0f;
float lastDesiredPitchRateDps = 0.0f;
float lastDesiredYawRateDps = 0.0f;
float lastCommandedRollDeg = 0.0f;
float lastCommandedPitchDeg = 0.0f;

// Uncorrected sample for CAL,START, kept before PidLink's stored offsets are
// applied so calibration never measures its own previous correction.
PidLinkImuSample baseImuSample;
bool haveBaseImuSample = false;

uint32_t validPidPacketCount = 0;
uint32_t invalidPidPacketCount = 0;

TraceSample traceRing[TRACE_RING_SAMPLES];
TraceState traceState = TRACE_IDLE;
uint16_t traceWriteIndex = 0;
uint16_t traceFilledSamples = 0;
uint16_t tracePostRemaining = 0;
uint16_t traceRearmCountdown = 0;
uint16_t traceCaptureId = 0;
uint16_t traceCaptureStart = 0;
uint16_t traceCaptureCount = 0;
uint16_t traceDrainIndex = 0;
uint32_t traceTriggerUs = 0;
uint8_t traceAxis = 0;
uint8_t traceInterleaveCounter = 0;
uint16_t traceDrainFailures = 0;
bool traceCaptureIsDisturbance = false;
bool lastMixSaturated = false;

bool radioOk = false;
bool imuInitialized = false;
bool imuOk = false;
bool imuCalibrating = false;
bool imuBiasValid = false;
bool imuSampleFresh = false;
uint32_t lastFreshImuUs = 0;
bool controlTimerOk = false;
bool failsafeActive = true;
FailsafeDescent failsafeDescent;
bool startupSafetyCleared = false;
bool batteryLow = false;
bool batteryCritical = false;

// startupSafetyCleared only says the preconditions hold. armed says the pilot
// actually asked for motor output; both are required before a motor turns.
bool armed = false;
bool armButtonPrev = false;
// Require a real released control frame before the first arm edge at boot and
// again after every direct STOP, remote STOP, or radio failsafe.
bool armButtonReleaseRequired = true;
bool calButtonPrev = true;
bool calButtonReleaseRequired = true;
uint32_t lastArmToggleMs = 0;

// Latched when a requested bias capture detects motion. A failed capture
// leaves imuBiasValid false, which blocks arming.
bool imuCalibrationMotionDetected = false;

uint32_t imuCalibrationSamples = 0;
float imuCalibrationGyroSpanDps = 0.0f;

float batteryVoltage = 0.0f;
float rollPidOutputUs = 0.0f;
float pitchPidOutputUs = 0.0f;
float yawPidOutputUs = 0.0f;

int16_t motorOutputUs[4] = {MOTOR_MIN_US, MOTOR_MIN_US, MOTOR_MIN_US, MOTOR_MIN_US};

uint32_t validPacketCount = 0;
uint32_t invalidPacketCount = 0;
uint32_t droppedSequenceCount = 0;
uint32_t telemetryQueueFailureCount = 0;
uint32_t lastValidPacketUs = 0;
uint16_t lastSequenceNumber = 0;
uint16_t nextTelemetrySequence = 0;
uint16_t nextImuTelemetrySequence = 0;
uint8_t wirelessImuInterleaveCounter = 0;
bool haveLastSequenceNumber = false;

volatile uint32_t scheduledControlTicks = 0;
uint32_t processedControlTicks = 0;
uint32_t skippedControlTicks = 0;
uint32_t controlExecutionOverruns = 0;
uint32_t controlDeadlineMisses = 0;
uint32_t controlLateStarts = 0;
uint32_t lastControlStartIntervalUs = 0;
uint32_t longestControlStartIntervalUs = 0;
uint32_t previousControlStartUs = 0;
uint32_t lastForegroundExecutionUs = 0;
uint32_t longestForegroundExecutionUs = 0;
uint32_t radioServiceUs = 0;
uint32_t longestRadioServiceUs = 0;
uint32_t lastImuReadUs = 0;
uint32_t longestImuReadUs = 0;
uint32_t lastControlExecutionUs = 0;
uint32_t longestControlExecutionUs = 0;
uint32_t lastBatteryUpdateMs = 0;
uint32_t lastDebugPwmUpdateMs = 0;
uint32_t lastLedUpdateMs = 0;
uint32_t lastSerialDebugMs = 0;

void controlTimerISR() {
  ++scheduledControlTicks;
}

// -----------------------------
// Utility helpers
// -----------------------------
int32_t clampInt32(int32_t value, int32_t low, int32_t high) {
  if (value < low) {
    return low;
  }
  if (value > high) {
    return high;
  }
  return value;
}

float clampFloat(float value, float low, float high) {
  if (value < low) {
    return low;
  }
  if (value > high) {
    return high;
  }
  return value;
}

float lowPassBlend(float dtSeconds, float timeConstantSeconds) {
  if (dtSeconds <= 0.0f) {
    return 0.0f;
  }
  return dtSeconds / (timeConstantSeconds + dtSeconds);
}

int32_t mapLinearClamped(int32_t value, int32_t inMin, int32_t inMax, int32_t outMin, int32_t outMax) {
  if (inMin == inMax) {
    return outMin;
  }

  value = clampInt32(value, min(inMin, inMax), max(inMin, inMax));
  return outMin + ((value - inMin) * (outMax - outMin)) / (inMax - inMin);
}

uint16_t computeChecksum(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;

  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if ((crc & 0x8000u) != 0) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021u);
      } else {
        crc = static_cast<uint16_t>(crc << 1);
      }
    }
  }

  return crc;
}

int16_t telemetrySigned(float value, float scale) {
  return static_cast<int16_t>(
      clampInt32(static_cast<int32_t>(lroundf(value * scale)), -32768, 32767));
}

uint16_t telemetryUnsigned(uint32_t value) {
  return static_cast<uint16_t>((value > 65535UL) ? 65535UL : value);
}

void fillTelemetryPacket(TelemetryPacket &outPacket) {
  uint8_t flags = 0;
  if (radioOk) {
    flags |= TELEMETRY_RADIO_OK;
  }
  if (imuOk) {
    flags |= TELEMETRY_IMU_OK;
  }
  if (controlTimerOk) {
    flags |= TELEMETRY_TIMER_OK;
  }
  if (failsafeActive) {
    flags |= TELEMETRY_FAILSAFE_ACTIVE;
  }
  if (startupSafetyCleared) {
    flags |= TELEMETRY_STARTUP_SAFETY_CLEARED;
  }
  if (batteryLow) {
    flags |= TELEMETRY_BATTERY_LOW;
  }
  if (batteryCritical) {
    flags |= TELEMETRY_BATTERY_CRITICAL;
  }
  if (angleModeEnabled) {
    flags |= TELEMETRY_ANGLE_MODE;
  }

  outPacket.protocolVersion = TELEMETRY_PROTOCOL_VERSION;
  outPacket.flags = flags;
  outPacket.telemetrySequence = nextTelemetrySequence;
  outPacket.rollCentideg = telemetrySigned(imu.rollDeg, 100.0f);
  outPacket.pitchCentideg = telemetrySigned(imu.pitchDeg, 100.0f);
  outPacket.gyroXDeciDps = telemetrySigned(imu.gxDps, 10.0f);
  outPacket.gyroYDeciDps = telemetrySigned(imu.gyDps, 10.0f);
  outPacket.gyroZDeciDps = telemetrySigned(imu.gzDps, 10.0f);
  outPacket.rateSetpointDeciDps[0] = telemetrySigned(lastDesiredRollRateDps, 10.0f);
  outPacket.rateSetpointDeciDps[1] = telemetrySigned(lastDesiredPitchRateDps, 10.0f);
  outPacket.rateSetpointDeciDps[2] = telemetrySigned(lastDesiredYawRateDps, 10.0f);
  outPacket.angleSetpointCentideg[0] = telemetrySigned(lastCommandedRollDeg, 100.0f);
  outPacket.angleSetpointCentideg[1] = telemetrySigned(lastCommandedPitchDeg, 100.0f);
  // Motor commands ride as (us - 1000) / 4 so all four fit in four bytes. The
  // transmitter reverses it; 4 us of resolution is far below anything visible
  // in a step response.
  for (uint8_t i = 0; i < 4; ++i) {
    const int32_t clamped = clampInt32(motorOutputUs[i], MOTOR_MIN_US, MOTOR_MAX_US);
    outPacket.motorQuarterUs[i] = static_cast<uint8_t>((clamped - MOTOR_MIN_US) / 4);
  }
  outPacket.batteryMillivolts = telemetryUnsigned(
      static_cast<uint32_t>(lroundf(max(batteryVoltage, 0.0f) * 1000.0f)));
  outPacket.checksum = 0;
  outPacket.checksum = computeChecksum(
      reinterpret_cast<const uint8_t *>(&outPacket),
      sizeof(TelemetryPacket) - sizeof(outPacket.checksum));
}

bool validatePacket(const ControlPacket &packet) {
  ControlPacket copy = packet;
  const uint16_t receivedChecksum = copy.checksum;
  copy.checksum = 0;

  const uint16_t expectedChecksum =
      computeChecksum(reinterpret_cast<const uint8_t *>(&copy), sizeof(ControlPacket) - sizeof(copy.checksum));

  if (receivedChecksum != expectedChecksum) {
    return false;
  }

  if (packet.rollCommand < COMMAND_MIN || packet.rollCommand > COMMAND_MAX) {
    return false;
  }
  if (packet.pitchCommand < COMMAND_MIN || packet.pitchCommand > COMMAND_MAX) {
    return false;
  }
  if (packet.yawCommand < COMMAND_MIN || packet.yawCommand > COMMAND_MAX) {
    return false;
  }
  if (packet.throttleCommand < THROTTLE_MIN_US || packet.throttleCommand > THROTTLE_MAX_US) {
    return false;
  }

  return true;
}

int16_t commandToRcPulseUs(int16_t command) {
  return static_cast<int16_t>(mapLinearClamped(command, COMMAND_MIN, COMMAND_MAX, THROTTLE_MIN_US, THROTTLE_MAX_US));
}

const char *radioDataRateLabel() {
  switch (RADIO_DATA_RATE) {
    case RF24_2MBPS:
      return "2M";
    case RF24_1MBPS:
      return "1M";
    case RF24_250KBPS:
      return "250K";
    default:
      return "?";
  }
}

// -----------------------------
// Radio
// -----------------------------
bool initRadio() {
  SPI.setMOSI(SPI_MOSI_PIN);
  SPI.setMISO(SPI_MISO_PIN);
  SPI.setSCK(SPI_SCK_PIN);
  SPI.begin();

  if (!radio.begin()) {
    return false;
  }

  radio.setChannel(RADIO_CHANNEL);
  radio.setAddressWidth(5);
  radio.setAutoAck(RF24_AUTO_ACK_ENABLED);
  radio.setRetries(RF24_RETRY_DELAY, RF24_RETRY_COUNT);
  radio.setDataRate(RADIO_DATA_RATE);
  radio.setPALevel(RADIO_PA_LEVEL);
  radio.setCRCLength(RF24_CRC_16);
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.openReadingPipe(1, RADIO_ADDRESS);
  radio.startListening();

  return radio.isChipConnected();
}

bool queueTelemetryAckPayload() {
  if (!radioOk) {
    return false;
  }

  fillTelemetryPacket(telemetryPacket);
  if (!radio.writeAckPayload(1, &telemetryPacket, sizeof(telemetryPacket))) {
    ++telemetryQueueFailureCount;
    return false;
  }

  ++nextTelemetrySequence;
  return true;
}

bool queueImuTelemetryAckPayload() {
  if (!radioOk) return false;

  imuTelemetryPacket = {};
  imuTelemetryPacket.magic = IMU_TELEMETRY_MAGIC;
  imuTelemetryPacket.protocolVersion = IMU_TELEMETRY_PROTOCOL_VERSION;
  imuTelemetryPacket.sequenceNumber = nextImuTelemetrySequence;
  imuTelemetryPacket.rollCentideg = telemetrySigned(imu.rollDeg, 100.0f);
  imuTelemetryPacket.pitchCentideg = telemetrySigned(imu.pitchDeg, 100.0f);
  imuTelemetryPacket.yawCentideg = telemetrySigned(imu.yawDeg, 100.0f);
  imuTelemetryPacket.gyroXDeciDps = telemetrySigned(imu.gxDps, 10.0f);
  imuTelemetryPacket.gyroYDeciDps = telemetrySigned(imu.gyDps, 10.0f);
  imuTelemetryPacket.gyroZDeciDps = telemetrySigned(imu.gzDps, 10.0f);
  imuTelemetryPacket.accelXMilliG = telemetrySigned(imu.axG, 1000.0f);
  imuTelemetryPacket.accelYMilliG = telemetrySigned(imu.ayG, 1000.0f);
  imuTelemetryPacket.accelZMilliG = telemetrySigned(imu.azG, 1000.0f);
  if (imuOk) imuTelemetryPacket.flags |= IMU_TELEMETRY_IMU_OK;
  if (controlTimerOk) imuTelemetryPacket.flags |= IMU_TELEMETRY_TIMER_OK;
  imuTelemetryPacket.checksum = computeChecksum(
      reinterpret_cast<const uint8_t *>(&imuTelemetryPacket),
      sizeof(imuTelemetryPacket) - sizeof(imuTelemetryPacket.checksum));

  if (!radio.writeAckPayload(1, &imuTelemetryPacket, sizeof(imuTelemetryPacket))) {
    ++telemetryQueueFailureCount;
    return false;
  }
  ++nextImuTelemetrySequence;
  return true;
}

// -----------------------------
// Live PID tuning, radio side
// -----------------------------
// Defined further down; needed by the link callbacks below.
bool motorOutputAllowed();
void writeAllMotorsMinimum();
void writeMotorOutputs();
void resetPid(PidState &state);
void resetDrehmControllerStates();
void resetPidTraceCapture();

bool pidGainsAreValid(float kp, float ki, float kd) {
  return isfinite(kp) && isfinite(ki) && isfinite(kd) &&
         kp >= 0.0f && kp <= PID_KP_MAX &&
         ki >= 0.0f && ki <= PID_KI_MAX &&
         kd >= 0.0f && kd <= PID_KD_MAX;
}

/* The outer loop's gains are on a different scale from the inner loop's - it
   outputs deg/s per degree, not microseconds per deg/s - so it gets its own
   bounds rather than borrowing the rate limits. */
bool angleGainsAreValid(float kp, float ki, float kd) {
  return isfinite(kp) && isfinite(ki) && isfinite(kd) &&
         kp >= 0.0f && kp <= ANGLE_KP_MAX &&
         ki >= 0.0f && ki <= ANGLE_KI_MAX &&
         kd >= 0.0f && kd <= ANGLE_KD_MAX;
}

bool isAngleControllerId(uint8_t id) {
  return id == PID_CONTROLLER_ROLL_ANGLE || id == PID_CONTROLLER_PITCH_ANGLE;
}

PidGains *runtimePidForController(uint8_t id) {
  switch (id) {
    case PID_CONTROLLER_ROLL: return &rollRatePid;
    case PID_CONTROLLER_PITCH: return &pitchRatePid;
    case PID_CONTROLLER_YAW: return &yawRatePid;
    case PID_CONTROLLER_ROLL_ANGLE: return &rollAnglePid;
    case PID_CONTROLLER_PITCH_ANGLE: return &pitchAnglePid;
    case PID_CONTROLLER_LEVEL_TRIM: return &levelTrimDeg;
    default: return nullptr;
  }
}

const PidGains *defaultPidForController(uint8_t id) {
  switch (id) {
    case PID_CONTROLLER_ROLL: return &DEFAULT_ROLL_RATE_PID;
    case PID_CONTROLLER_PITCH: return &DEFAULT_PITCH_RATE_PID;
    case PID_CONTROLLER_YAW: return &DEFAULT_YAW_RATE_PID;
    case PID_CONTROLLER_ROLL_ANGLE: return &DEFAULT_ROLL_ANGLE_PID;
    case PID_CONTROLLER_PITCH_ANGLE: return &DEFAULT_PITCH_ANGLE_PID;
    case PID_CONTROLLER_LEVEL_TRIM: return &DEFAULT_LEVEL_TRIM;
    default: return nullptr;
  }
}

PidState *pidStateForController(uint8_t id) {
  switch (id) {
    case PID_CONTROLLER_ROLL: return &rollPidState;
    case PID_CONTROLLER_PITCH: return &pitchPidState;
    case PID_CONTROLLER_YAW: return &yawPidState;
    case PID_CONTROLLER_ROLL_ANGLE: return &rollAnglePidState;
    case PID_CONTROLLER_PITCH_ANGLE: return &pitchAnglePidState;
    case PID_CONTROLLER_LEVEL_TRIM: return &levelTrimUnusedState;
    default: return nullptr;
  }
}

/* Level offsets are signed, unlike gains. The unused third payload value must
   remain zero so stale three-motor trim commands are rejected. */
bool levelTrimIsValid(float rollDeg, float pitchDeg, float unused) {
  return isfinite(rollDeg) && isfinite(pitchDeg) && isfinite(unused) &&
         fabsf(rollDeg) <= LEVEL_TRIM_LIMIT_DEG &&
         fabsf(pitchDeg) <= LEVEL_TRIM_LIMIT_DEG &&
         fabsf(unused) <= 1e-6f;
}

bool controllerGainsAreValid(uint8_t id, float kp, float ki, float kd) {
  if (id == PID_CONTROLLER_LEVEL_TRIM) {
    return levelTrimIsValid(kp, ki, kd);
  }
  return isAngleControllerId(id) ? angleGainsAreValid(kp, ki, kd)
                                 : pidGainsAreValid(kp, ki, kd);
}

/* dRehm stores raw error*time, so it must be rescaled across a nonzero Ki
   change to retain its I contribution. A zero Ki deliberately clears it. The
   legacy filteredDerivativeTerm is unused by this controller; preserving the
   prior error prevents a gain-only derivative kick. */
void applyPidGainsBumpless(PidGains &runtimeGains,
                           const PidGains &newGains,
                           PidState &state) {
  if (&state == &levelTrimUnusedState) {
    state.integrator = 0.0f;
  } else if (runtimeGains.ki > 1e-6f && newGains.ki > 1e-6f) {
    state.integrator = clampFloat(state.integrator * runtimeGains.ki / newGains.ki,
                                  -DREHM_INTEGRATOR_RAW_LIMIT,
                                  DREHM_INTEGRATOR_RAW_LIMIT);
  } else {
    state.integrator = 0.0f;
  }
  state.filteredDerivativeTerm = 0.0f;
  runtimeGains = newGains;
}

bool validatePidUpdatePacket(const PidUpdatePacket &packet) {
  if (packet.magic != PID_UPDATE_MAGIC) return false;
  PidUpdatePacket copy = packet;
  const uint16_t received = copy.checksum;
  copy.checksum = 0;
  return received == computeChecksum(reinterpret_cast<const uint8_t *>(&copy),
                                     sizeof(PidUpdatePacket) - sizeof(copy.checksum));
}

void fillPidStatusPacket(PidStatusPacket &outPacket,
                         const PidUpdatePacket &request,
                         uint8_t statusCode,
                         const PidGains &gains) {
  outPacket.magic = PID_STATUS_MAGIC;
  outPacket.status = statusCode;
  outPacket.controllerId = request.controllerId;
  outPacket.command = request.command;
  outPacket.sequenceNumber = request.sequenceNumber;
  outPacket.kp = gains.kp;
  outPacket.ki = gains.ki;
  outPacket.kd = gains.kd;
  outPacket.checksum = 0;
  outPacket.checksum = computeChecksum(reinterpret_cast<const uint8_t *>(&outPacket),
                                       sizeof(PidStatusPacket) - sizeof(outPacket.checksum));
}

bool queuePidStatusAckPayload(const PidStatusPacket &packet) {
  if (!radioOk) return false;
  if (!radio.writeAckPayload(1, &packet, sizeof(packet))) {
    ++telemetryQueueFailureCount;
    return false;
  }
  return true;
}

void processPidUpdatePacket(const PidUpdatePacket &request) {
  PidStatusPacket response = {};
  PidGains fallback = {0.0f, 0.0f, 0.0f};

  if (!validatePidUpdatePacket(request)) {
    ++invalidPidPacketCount;
    fillPidStatusPacket(response, request, PID_STATUS_INVALID_PACKET, fallback);
    queuePidStatusAckPayload(response);
    return;
  }

  PidGains *runtimeGains = runtimePidForController(request.controllerId);
  const PidGains *defaultGains = defaultPidForController(request.controllerId);
  PidState *state = pidStateForController(request.controllerId);
  if (runtimeGains == nullptr || defaultGains == nullptr || state == nullptr) {
    ++invalidPidPacketCount;
    fillPidStatusPacket(response, request, PID_STATUS_UNKNOWN_CONTROLLER, fallback);
    queuePidStatusAckPayload(response);
    return;
  }

  uint8_t status = PID_STATUS_UNKNOWN_COMMAND;
  if (request.command == PID_COMMAND_SET) {
    if (!controllerGainsAreValid(request.controllerId, request.kp, request.ki, request.kd)) {
      ++invalidPidPacketCount;
      fillPidStatusPacket(response, request, PID_STATUS_INVALID_VALUES, *runtimeGains);
      queuePidStatusAckPayload(response);
      return;
    }
    const PidGains requested = {request.kp, request.ki, request.kd};
    applyPidGainsBumpless(*runtimeGains, requested, *state);
    // Never label samples from two different gain sets as one capture.
    resetPidTraceCapture();
    status = PID_STATUS_APPLIED;
  } else if (request.command == PID_COMMAND_GET) {
    status = PID_STATUS_CURRENT;
  } else if (request.command == PID_COMMAND_RESET) {
    applyPidGainsBumpless(*runtimeGains, *defaultGains, *state);
    resetPidTraceCapture();
    status = PID_STATUS_RESET_TO_DEFAULT;
  } else {
    ++invalidPidPacketCount;
  }

  ++validPidPacketCount;
  fillPidStatusPacket(response, request, status, *runtimeGains);
  queuePidStatusAckPayload(response);
}

// ---------------------------------------------------------------------------
// Passive onboard rate-loop response capture
// ---------------------------------------------------------------------------
uint16_t traceRingIndexBack(uint16_t fromIndex, uint16_t stepsBack) {
  return static_cast<uint16_t>(
      (fromIndex + TRACE_RING_SAMPLES - stepsBack) % TRACE_RING_SAMPLES);
}

void resetPidTraceCapture() {
  if (traceState != TRACE_IDLE) {
    // If any part reached the browser, the next capture must have a new ID.
    ++traceCaptureId;
  }
  traceState = TRACE_IDLE;
  traceWriteIndex = 0;
  traceFilledSamples = 0;
  tracePostRemaining = 0;
  traceRearmCountdown = 0;
  traceCaptureStart = 0;
  traceCaptureCount = 0;
  traceDrainIndex = 0;
  traceTriggerUs = 0;
  traceAxis = 0;
  traceInterleaveCounter = 0;
  traceDrainFailures = 0;
  traceCaptureIsDisturbance = false;
}

void finishPidTraceDrain(bool abandoned) {
  (void)abandoned;  // retained for readable call sites and future diagnostics
  traceState = TRACE_IDLE;
  traceWriteIndex = 0;
  traceFilledSamples = 0;
  tracePostRemaining = 0;
  traceCaptureStart = 0;
  traceCaptureCount = 0;
  traceDrainIndex = 0;
  traceInterleaveCounter = 0;
  traceDrainFailures = 0;
  traceCaptureIsDisturbance = false;
  traceRearmCountdown = TRACE_REARM_SAMPLES;
  ++traceCaptureId;
}

void recordPidTraceSample(float desiredRollDps,
                          float desiredPitchDps,
                          float desiredYawDps) {
  TraceSample &slot = traceRing[traceWriteIndex];
  slot.timestampUs = micros();

  slot.setpointDeciDps[PID_CONTROLLER_ROLL] = telemetrySigned(desiredRollDps, 10.0f);
  slot.setpointDeciDps[PID_CONTROLLER_PITCH] = telemetrySigned(desiredPitchDps, 10.0f);
  slot.setpointDeciDps[PID_CONTROLLER_YAW] = telemetrySigned(desiredYawDps, 10.0f);
  slot.measuredDeciDps[PID_CONTROLLER_ROLL] = telemetrySigned(imu.gxDps, 10.0f);
  slot.measuredDeciDps[PID_CONTROLLER_PITCH] = telemetrySigned(imu.gyDps, 10.0f);
  slot.measuredDeciDps[PID_CONTROLLER_YAW] = telemetrySigned(imu.gzDps, 10.0f);

  const PidState *states[3] = {&rollPidState, &pitchPidState, &yawPidState};
  slot.termClippedMask = 0;
  for (uint8_t axis = 0; axis < 3; ++axis) {
    // The established wire format is signed centi-microseconds. Flag, rather
    // than silently hiding, any term outside its +/-327.67 us range.
    if (fabsf(states[axis]->pTerm) > 327.67f ||
        fabsf(states[axis]->iTerm) > 327.67f ||
        fabsf(states[axis]->dTerm) > 327.67f) {
      slot.termClippedMask |= static_cast<uint8_t>(1u << axis);
    }
    slot.pTermCentiUs[axis] = telemetrySigned(states[axis]->pTerm, 100.0f);
    slot.iTermCentiUs[axis] = telemetrySigned(states[axis]->iTerm, 100.0f);
    slot.dTermCentiUs[axis] = telemetrySigned(states[axis]->dTerm, 100.0f);
  }
  slot.motorSaturated = lastMixSaturated ? 1u : 0u;

  traceWriteIndex = static_cast<uint16_t>((traceWriteIndex + 1u) % TRACE_RING_SAMPLES);
  if (traceFilledSamples < TRACE_RING_SAMPLES) ++traceFilledSamples;
}

// Called only after a completed mix on every fourth scheduled 2 kHz tick.
// It observes existing state. It never writes a receiver command, setpoint,
// PID contribution, mixer value, motor value, or output peripheral.
void updatePidTraceCapture(float desiredRollDps,
                           float desiredPitchDps,
                           float desiredYawDps) {
  if (!motorOutputAllowed() ||
      receiverCommand.throttle <= THROTTLE_LOW_CUTOFF_US) {
    resetPidTraceCapture();
    return;
  }

  // The captured ring is frozen while it drains. Recording here would overwrite
  // samples before the slow radio downlink had transmitted them.
  if (traceState == TRACE_DRAINING) return;

  if (traceRearmCountdown > 0) --traceRearmCountdown;
  recordPidTraceSample(desiredRollDps, desiredPitchDps, desiredYawDps);

  if (traceState == TRACE_CAPTURING) {
    if (tracePostRemaining > 0) --tracePostRemaining;
    if (tracePostRemaining == 0) {
      traceCaptureCount = TRACE_TOTAL_SAMPLES;
      traceCaptureStart = traceRingIndexBack(traceWriteIndex, traceCaptureCount);
      traceDrainIndex = 0;
      traceInterleaveCounter = 0;
      traceState = TRACE_DRAINING;
    }
    return;
  }

  if (traceRearmCountdown > 0 ||
      traceFilledSamples < TRACE_PRE_SAMPLES + TRACE_TRIGGER_LOOKBACK) {
    return;
  }

  const uint16_t latest = traceRingIndexBack(traceWriteIndex, 1);
  const uint16_t earlier =
      traceRingIndexBack(traceWriteIndex, 1 + TRACE_TRIGGER_LOOKBACK);

  float largestDelta = 0.0f;
  uint8_t steppedAxis = PID_CONTROLLER_ROLL;
  for (uint8_t axis = 0; axis < 3; ++axis) {
    const float delta = fabsf(
        static_cast<float>(traceRing[latest].setpointDeciDps[axis] -
                           traceRing[earlier].setpointDeciDps[axis]) / 10.0f);
    if (delta > largestDelta) {
      largestDelta = delta;
      steppedAxis = axis;
    }
  }

  if (largestDelta >= TRACE_TRIGGER_DELTA_DPS) {
    traceAxis = steppedAxis;
    traceCaptureIsDisturbance = false;
    traceTriggerUs = traceRing[latest].timestampUs;
    tracePostRemaining = TRACE_POST_SAMPLES - 1u;
    traceState = TRACE_CAPTURING;
    return;
  }

  // With a steady commanded rate, a large actual-rate error is treated as a
  // natural disturbance. This still injects nothing into the flight path.
  if (largestDelta > TRACE_DISTURBANCE_STEADY_DPS) return;

  float largestError = 0.0f;
  uint8_t disturbedAxis = PID_CONTROLLER_ROLL;
  for (uint8_t axis = 0; axis < 3; ++axis) {
    const float error = fabsf(
        static_cast<float>(traceRing[latest].measuredDeciDps[axis] -
                           traceRing[latest].setpointDeciDps[axis]) / 10.0f);
    if (error > largestError) {
      largestError = error;
      disturbedAxis = axis;
    }
  }
  if (largestError < TRACE_DISTURBANCE_ERROR_DPS) return;

  traceAxis = disturbedAxis;
  traceCaptureIsDisturbance = true;
  traceTriggerUs = traceRing[latest].timestampUs;
  tracePostRemaining = TRACE_POST_SAMPLES - 1u;
  traceState = TRACE_CAPTURING;
}

bool queuePidTraceAckPayload() {
  if (traceState != TRACE_DRAINING || traceDrainIndex >= traceCaptureCount) {
    return false;
  }
  if (!radioOk) {
    if (++traceDrainFailures >= TRACE_DRAIN_MAX_FAILURES) {
      finishPidTraceDrain(true);
    }
    return false;
  }

  const uint16_t ringIndex = static_cast<uint16_t>(
      (traceCaptureStart + traceDrainIndex) % TRACE_RING_SAMPLES);
  const TraceSample &sample = traceRing[ringIndex];

  PidTracePacket packet = {};
  packet.magic = PID_TRACE_MAGIC;
  packet.axisId = traceAxis;
  if (traceDrainIndex == 0) packet.flags |= PID_TRACE_FIRST_SAMPLE;
  if (traceDrainIndex + 1u == traceCaptureCount) packet.flags |= PID_TRACE_LAST_SAMPLE;
  if (sample.motorSaturated) packet.flags |= PID_TRACE_MOTOR_SATURATED;
  if (traceCaptureIsDisturbance) packet.flags |= PID_TRACE_DISTURBANCE;
  if ((sample.termClippedMask & static_cast<uint8_t>(1u << traceAxis)) != 0u) {
    packet.flags |= PID_TRACE_TERM_CLIPPED;
  }
  packet.captureId = traceCaptureId;
  packet.sampleIndex = traceDrainIndex;
  packet.sampleCount = traceCaptureCount;
  const int32_t relativeUs = static_cast<int32_t>(sample.timestampUs - traceTriggerUs);
  packet.relativeMs = static_cast<int16_t>(
      clampInt32(relativeUs / 1000, -32768, 32767));
  packet.setpointDeciDps = sample.setpointDeciDps[traceAxis];
  packet.measuredDeciDps = sample.measuredDeciDps[traceAxis];
  packet.pTermCentiUs = sample.pTermCentiUs[traceAxis];
  packet.iTermCentiUs = sample.iTermCentiUs[traceAxis];
  packet.dTermCentiUs = sample.dTermCentiUs[traceAxis];
  packet.checksum = computeChecksum(
      reinterpret_cast<const uint8_t *>(&packet),
      sizeof(packet) - sizeof(packet.checksum));

  if (!radio.writeAckPayload(1, &packet, sizeof(packet))) {
    ++telemetryQueueFailureCount;
    if (++traceDrainFailures >= TRACE_DRAIN_MAX_FAILURES) {
      finishPidTraceDrain(true);
    }
    return false;
  }

  traceDrainFailures = 0;
  ++traceDrainIndex;
  if (traceDrainIndex >= traceCaptureCount) finishPidTraceDrain(false);
  return true;
}

/* Announce identity every couple of seconds. The transmitter turns this into a
   $I line and the tuner matches the hash against its descriptor cache. */
uint32_t lastIdentitySentMs = 0;
constexpr uint32_t IDENTITY_INTERVAL_MS = 2000;

bool queueIdentityAckPayload() {
  if (!radioOk) return false;
  PidIdentityPacket p = {};
  p.magic = PID_IDENTITY_MAGIC;
  p.descriptorHash = link.descriptorHash();
  const size_t nameLen = min(strlen(link.firmwareName()), sizeof(p.name));
  const size_t versionLen = min(strlen(link.firmwareVersion()), sizeof(p.version));
  memcpy(p.name, link.firmwareName(), nameLen);
  memcpy(p.version, link.firmwareVersion(), versionLen);
  p.nameLen = static_cast<uint8_t>(nameLen);
  p.checksum = 0;
  p.checksum = computeChecksum(reinterpret_cast<const uint8_t *>(&p),
                               sizeof(p) - sizeof(p.checksum));
  if (!radio.writeAckPayload(1, &p, sizeof(p))) {
    ++telemetryQueueFailureCount;
    return false;
  }
  lastIdentitySentMs = millis();
  return true;
}

bool validateDescriptorRequestPacket(const PidDescriptorRequestPacket &candidate) {
  if (candidate.magic != PID_DESCRIPTOR_REQUEST_MAGIC) return false;
  PidDescriptorRequestPacket copy = candidate;
  const uint16_t received = copy.checksum;
  copy.checksum = 0;
  return received == computeChecksum(reinterpret_cast<const uint8_t *>(&copy),
                                     sizeof(PidDescriptorRequestPacket) - sizeof(copy.checksum));
}

bool queueDescriptorChunkAckPayload(const PidDescriptorRequestPacket &request) {
  if (!radioOk || request.descriptorHash != link.descriptorHash()) return false;
  const size_t descriptorLen = sizeof(DESCRIPTOR) - 1u;
  const uint16_t total = static_cast<uint16_t>(
      (descriptorLen + PID_DESCRIPTOR_DATA_BYTES - 1u) / PID_DESCRIPTOR_DATA_BYTES);
  if (total == 0 || total > PID_DESCRIPTOR_MAX_CHUNKS || request.index >= total) return false;

  PidDescriptorChunkPacket packet = {};
  packet.magic = PID_DESCRIPTOR_CHUNK_MAGIC;
  packet.index = request.index;
  packet.total = total;
  packet.descriptorHash = link.descriptorHash();
  const size_t offset = static_cast<size_t>(request.index) * PID_DESCRIPTOR_DATA_BYTES;
  const size_t remaining = descriptorLen - offset;
  packet.length = static_cast<uint8_t>(
      remaining < PID_DESCRIPTOR_DATA_BYTES ? remaining : PID_DESCRIPTOR_DATA_BYTES);
  memcpy(packet.data, DESCRIPTOR + offset, packet.length);
  packet.checksum = 0;
  packet.checksum = computeChecksum(reinterpret_cast<const uint8_t *>(&packet),
                                    sizeof(PidDescriptorChunkPacket) - sizeof(packet.checksum));

  // Descriptor transfer is request/response traffic. Clear stale telemetry ACK
  // payloads so the requested chunk is the next payload the transmitter sees.
  radio.flush_tx();
  if (!radio.writeAckPayload(1, &packet, sizeof(packet))) {
    ++telemetryQueueFailureCount;
    return false;
  }
  return true;
}

bool queueDownlinkAckPayload() {
  // Identity keeps its original two-second priority even during a trace drain.
  // PID status and descriptor chunks are queued directly by their request
  // handlers, so this passive stream cannot replace those responses.
  if ((uint32_t)(millis() - lastIdentitySentMs) >= IDENTITY_INTERVAL_MS) {
    if (queueIdentityAckPayload()) return true;
  }
  if (traceState == TRACE_DRAINING) {
    ++traceInterleaveCounter;
    if ((traceInterleaveCounter % TRACE_TELEMETRY_INTERLEAVE) != 0u) {
      if (queuePidTraceAckPayload()) return true;
    }
  } else {
    // Control packets arrive at 100 Hz. One dedicated attitude ACK out of four
    // gives the renderer a smooth 25 Hz stream; ordinary tuner telemetry uses
    // the other slots and retains priority during PID trace capture.
    ++wirelessImuInterleaveCounter;
    if ((wirelessImuInterleaveCounter % 4u) == 0u) {
      if (queueImuTelemetryAckPayload()) return true;
    }
  }
  return queueTelemetryAckPayload();
}

void updateReceiver() {
  if (!radioOk) {
    return;
  }

  uint8_t servicedPackets = 0;
  while (servicedPackets < 3u && radio.available()) {
    ++servicedPackets;
    const uint8_t payloadSize = radio.getDynamicPayloadSize();
    if (payloadSize == 0 || payloadSize > 32) {
      ++invalidPacketCount;
      radio.flush_rx();
      queueDownlinkAckPayload();
      continue;
    }

    if (payloadSize == sizeof(ControlPacket)) {
      ControlPacket candidate = {};
      radio.read(&candidate, sizeof(candidate));

      if (!validatePacket(candidate)) {
        ++invalidPacketCount;
      } else {
        if (haveLastSequenceNumber) {
          const uint16_t gap = static_cast<uint16_t>(candidate.sequenceNumber - lastSequenceNumber);
          if (gap > 1 && gap < 32768u) {
            droppedSequenceCount += static_cast<uint32_t>(gap - 1u);
          }
        }

        haveLastSequenceNumber = true;
        lastSequenceNumber = candidate.sequenceNumber;
        latestPacket = candidate;

        receiverCommand.roll = candidate.rollCommand;
        receiverCommand.pitch = candidate.pitchCommand;
        receiverCommand.yaw = candidate.yawCommand;
        receiverCommand.throttle = candidate.throttleCommand;
        receiverCommand.buttons = candidate.buttons;
        receiverCommand.flags = candidate.flags;
        receiverCommand.sequenceNumber = candidate.sequenceNumber;

        if ((candidate.buttons & CONTROL_BUTTON_REMOTE_STOP_MASK) != 0u) {
          failsafeDescent.cancel();
          // The bridge forces ARM high in this STOP frame. Keep that value for
          // updateArming(), so the STOP frame itself cannot masquerade as the
          // required release. Only a later ordinary released frame clears it.
          receiverCommand.roll = FAILSAFE_ROLL;
          receiverCommand.pitch = FAILSAFE_PITCH;
          receiverCommand.yaw = FAILSAFE_YAW;
          receiverCommand.throttle = FAILSAFE_THROTTLE_US;
          receiverCommand.buttons = CONTROL_BUTTON_ARM_MASK;
          armed = false;
          startupSafetyCleared = false;
          armButtonPrev = true;
          armButtonReleaseRequired = true;
          calButtonPrev = true;
          calButtonReleaseRequired = true;
          resetDrehmControllerStates();
          resetPidTraceCapture();
          writeAllMotorsMinimum();
          writeMotorOutputs();
        }

        ++validPacketCount;
        lastValidPacketUs = micros();
      }

      // This payload is returned with the next control packet's hardware ACK.
      queueDownlinkAckPayload();
    } else if (payloadSize == sizeof(PidUpdatePacket)) {
      PidUpdatePacket request = {};
      radio.read(&request, sizeof(request));
      // Tuning traffic intentionally does not refresh the flight-control
      // failsafe. Only a control packet counts as the pilot still being there.
      processPidUpdatePacket(request);
    } else if (payloadSize == sizeof(PidDescriptorRequestPacket)) {
      PidDescriptorRequestPacket request = {};
      radio.read(&request, sizeof(request));
      // A descriptor request does not refresh the flight-control failsafe.
      // The tuner-aware transmitter interleaves ordinary control packets while
      // discovery runs; those controls keep a healthy link alive. If they stop,
      // the normal 150 ms failsafe still disarms the aircraft.
      if (validateDescriptorRequestPacket(request)) {
        if (!queueDescriptorChunkAckPayload(request)) ++invalidPacketCount;
      } else {
        ++invalidPacketCount;
        queueDownlinkAckPayload();
      }
    } else {
      uint8_t discardedPayload[32];
      radio.read(discardedPayload, payloadSize);
      ++invalidPacketCount;
      queueDownlinkAckPayload();
    }
  }
}

// ---------------------------------------------------------------------------
// PID Tuner Protocol over this board's own USB port
//
// Independent of the radio path: the transmitter keeps using the binary packets
// above while flying. This is what you plug into on the bench, and what supplies
// the descriptor the tuner caches.
// ---------------------------------------------------------------------------
uint32_t usbTelemetryDivider = CONTROL_LOOP_HZ / 25;
uint32_t usbTelemetryCounter = 0;

PidGains *gainsByName(const char *id) {
  if (strcmp(id, "roll") == 0) return &rollRatePid;
  if (strcmp(id, "pitch") == 0) return &pitchRatePid;
  if (strcmp(id, "yaw") == 0) return &yawRatePid;
  if (strcmp(id, "roll_angle") == 0) return &rollAnglePid;
  if (strcmp(id, "pitch_angle") == 0) return &pitchAnglePid;
  if (strcmp(id, "level_trim") == 0) return &levelTrimDeg;
  return nullptr;
}

const PidGains *defaultsByName(const char *id) {
  if (strcmp(id, "roll") == 0) return &DEFAULT_ROLL_RATE_PID;
  if (strcmp(id, "pitch") == 0) return &DEFAULT_PITCH_RATE_PID;
  if (strcmp(id, "yaw") == 0) return &DEFAULT_YAW_RATE_PID;
  if (strcmp(id, "roll_angle") == 0) return &DEFAULT_ROLL_ANGLE_PID;
  if (strcmp(id, "pitch_angle") == 0) return &DEFAULT_PITCH_ANGLE_PID;
  if (strcmp(id, "level_trim") == 0) return &DEFAULT_LEVEL_TRIM;
  return nullptr;
}

PidState *stateByName(const char *id) {
  if (strcmp(id, "roll") == 0) return &rollPidState;
  if (strcmp(id, "pitch") == 0) return &pitchPidState;
  if (strcmp(id, "yaw") == 0) return &yawPidState;
  if (strcmp(id, "roll_angle") == 0) return &rollAnglePidState;
  if (strcmp(id, "pitch_angle") == 0) return &pitchAnglePidState;
  if (strcmp(id, "level_trim") == 0) return &levelTrimUnusedState;
  return nullptr;
}

bool isAngleControllerName(const char *id) {
  return strcmp(id, "roll_angle") == 0 || strcmp(id, "pitch_angle") == 0;
}

bool isLevelTrimName(const char *id) {
  return strcmp(id, "level_trim") == 0;
}

void replyGainsFor(const char *id, uint8_t code) {
  const PidGains *g = gainsByName(id);
  link.pidReplyBegin(id, code);
  if (g) {
    link.pidReplyParam("kp", g->kp);
    link.pidReplyParam("ki", g->ki);
    link.pidReplyParam("kd", g->kd);
  }
  link.pidReplyEnd();
}

void linkPidGet(const char *id) {
  if (!gainsByName(id)) {
    link.pidReplyBegin(id, PIDLINK_UNKNOWN_CONTROLLER);
    link.pidReplyEnd();
    return;
  }
  replyGainsFor(id, PIDLINK_CURRENT);
}

void linkPidReset(const char *id) {
  PidGains *g = gainsByName(id);
  const PidGains *d = defaultsByName(id);
  PidState *s = stateByName(id);
  if (!g || !d || !s) {
    link.pidReplyBegin(id, PIDLINK_UNKNOWN_CONTROLLER);
    link.pidReplyEnd();
    return;
  }
  applyPidGainsBumpless(*g, *d, *s);
  resetPidTraceCapture();
  replyGainsFor(id, PIDLINK_RESET);
}

void linkPidSet(const char *id, const PidLinkParams &p) {
  PidGains *g = gainsByName(id);
  PidState *s = stateByName(id);
  if (!g || !s) {
    link.pidReplyBegin(id, PIDLINK_UNKNOWN_CONTROLLER);
    link.pidReplyEnd();
    return;
  }
  if (p.anyMalformed()) { replyGainsFor(id, PIDLINK_REJECTED); return; }
  // Start from the live values so a partial SET changes only what it names.
  float kp = g->kp, ki = g->ki, kd = g->kd;
  p.get("kp", kp);
  p.get("ki", ki);
  p.get("kd", kd);
  const bool ok = isLevelTrimName(id)
                      ? levelTrimIsValid(kp, ki, kd)
                      : isAngleControllerName(id) ? angleGainsAreValid(kp, ki, kd)
                                                  : pidGainsAreValid(kp, ki, kd);
  if (!ok) { replyGainsFor(id, PIDLINK_REJECTED); return; }
  const PidGains requested = {kp, ki, kd};
  applyPidGainsBumpless(*g, requested, *s);
  resetPidTraceCapture();
  replyGainsFor(id, PIDLINK_APPLIED);
}

/* PidLink must see the sample before its own stored offsets are applied,
   otherwise a calibration would be measuring its own previous correction. The
   manually requested calibration supplies the base bias; a saved
   tuner calibration adds a persistent residual on top. */
bool linkImuSample(PidLinkImuSample &out) {
  if (!imuInitialized || !imuOk || !imuBiasValid || imuCalibrating || !haveBaseImuSample) return false;
  out = baseImuSample;
  return true;
}

/* Motors must be genuinely idle before calibrating: prop wash and frame
   vibration would be averaged straight into the offsets. */
bool linkIsStopped() {
  return !armed && !imuCalibrating;
}

void linkStop() {
  failsafeDescent.cancel();
  // STOP is a disarm, not merely a startup-gate reset. link.poll() runs before
  // updateReceiver()/updateArming(), so consume a held or same-loop-arriving
  // arm button and require a release followed by a new low-throttle edge.
  noInterrupts();
  armed = false;
  startupSafetyCleared = false;
  armButtonPrev = true;
  armButtonReleaseRequired = true;
  calButtonPrev = true;
  calButtonReleaseRequired = true;
  interrupts();
  resetDrehmControllerStates();
  resetPidTraceCapture();
  writeAllMotorsMinimum();
  writeMotorOutputs();
  link.state(true, false, "STOP disarmed; fresh explicit low-throttle arm action required");
}

bool linkTelemetryRate(uint32_t hz) {
  if (hz == 0 || hz > CONTROL_LOOP_HZ || (CONTROL_LOOP_HZ % hz) != 0) return false;
  usbTelemetryDivider = CONTROL_LOOP_HZ / hz;
  return true;
}

/* Field order must match telemetry.fields in DESCRIPTOR, and must match what
   the transmitter prints when flying. All three are one contract. */
void emitImu3dTelemetry() {
  // Sensor-neutral, receive-only stream consumed by IMU 3D Live Test when this
  // flight-controller Teensy is connected directly over USB. It is deliberately
  // separate from $T so the established tuner descriptor remains unchanged.
  // MPU6050 acceleration is stored in g; the live-page contract uses m/s^2.
  constexpr float STANDARD_GRAVITY_MPS2 = 9.80665f;
  usbProtocol.print(F("IMU,FC_USB,"));
  usbProtocol.print(imu.rollDeg, 3);
  usbProtocol.print(','); usbProtocol.print(imu.pitchDeg, 3);
  usbProtocol.print(','); usbProtocol.print(imu.yawDeg, 3);
  usbProtocol.print(','); usbProtocol.print(imu.gxDps, 3);
  usbProtocol.print(','); usbProtocol.print(imu.gyDps, 3);
  usbProtocol.print(','); usbProtocol.print(imu.gzDps, 3);
  usbProtocol.print(','); usbProtocol.print(imu.axG * STANDARD_GRAVITY_MPS2, 3);
  usbProtocol.print(','); usbProtocol.print(imu.ayG * STANDARD_GRAVITY_MPS2, 3);
  usbProtocol.print(','); usbProtocol.println(imu.azG * STANDARD_GRAVITY_MPS2, 3);
}

void emitUsbTelemetry() {
  if (!usbProtocol || usbProtocol.availableForWrite() < 512) return;
  link.telemetryBegin();
  link.telemetryValue((uint32_t)micros());
  link.telemetryValue(imu.gxDps, 2);
  link.telemetryValue(lastDesiredRollRateDps, 2);
  link.telemetryValue(imu.gyDps, 2);
  link.telemetryValue(lastDesiredPitchRateDps, 2);
  link.telemetryValue(imu.gzDps, 2);
  link.telemetryValue(lastDesiredYawRateDps, 2);
  for (uint8_t i = 0; i < 4; ++i) link.telemetryValue((long)motorOutputUs[i]);
  link.telemetryValue((long)receiverCommand.roll);
  link.telemetryValue((long)receiverCommand.pitch);
  link.telemetryValue((long)receiverCommand.yaw);
  link.telemetryValue((long)receiverCommand.throttle);
  link.telemetryValue(imu.rollDeg, 2);
  link.telemetryValue(lastCommandedRollDeg, 2);
  link.telemetryValue(imu.pitchDeg, 2);
  link.telemetryValue(lastCommandedPitchDeg, 2);
  link.telemetryValue(angleModeEnabled);
  link.telemetryValue(batteryVoltage, 2);
  link.telemetryValue(failsafeActive);
  link.telemetryValue(batteryLow);
  link.telemetryValue(!radioOk);
  link.telemetryEnd();
  emitImu3dTelemetry();
}

void initPidLink() {
  usbProtocol.begin(115200);
  link.begin(DESCRIPTOR);
  link.onPidSet(linkPidSet);
  link.onPidGet(linkPidGet);
  link.onPidReset(linkPidReset);
  link.onImuSample(linkImuSample);
  link.onIsStopped(linkIsStopped);
  link.onStop(linkStop);
  link.onTelemetryRate(linkTelemetryRate);
}

void updateFailsafe(uint32_t nowUs) {
  const bool nextFailsafeActive =
      (lastValidPacketUs == 0) || (nowUs - lastValidPacketUs > FAILSAFE_TIMEOUT_US);
  if (nextFailsafeActive && !failsafeActive) {
    // Enter only from controlled flight. A missing level or rate controller
    // cannot self-level, so it must fail closed.
    const bool levelControlUsable =
        rollAnglePid.kp > 0.0f && pitchAnglePid.kp > 0.0f &&
        rollRatePid.kp > 0.0f && pitchRatePid.kp > 0.0f;
    const bool healthy =
        imuInitialized && imuOk && imuBiasValid && !imuCalibrating &&
        !link.calibrationRunning() && controlTimerOk && startupSafetyCleared &&
        isfinite(imu.rollDeg) && isfinite(imu.pitchDeg) &&
        isfinite(imu.gxDps) && isfinite(imu.gyDps) && isfinite(imu.gzDps) &&
        !(DISABLE_MOTORS_ON_CRITICAL_BATTERY && batteryCritical);
    if (armed && receiverCommand.throttle > THROTTLE_LOW_CUTOFF_US &&
        healthy && levelControlUsable) {
      failsafeDescent.start(nowUs, receiverCommand.throttle);
    } else {
      armed = false;
      resetDrehmControllerStates();
      resetPidTraceCapture();
    }
    armButtonPrev = true;
    armButtonReleaseRequired = true;
    calButtonPrev = true;
    calButtonReleaseRequired = true;
  }
  failsafeActive = nextFailsafeActive;

  if (failsafeDescent.active()) {
    const bool healthy =
        armed && imuInitialized && imuOk && imuBiasValid && !imuCalibrating &&
        !link.calibrationRunning() && controlTimerOk &&
        isfinite(imu.rollDeg) && isfinite(imu.pitchDeg) &&
        isfinite(imu.gxDps) && isfinite(imu.gyDps) && isfinite(imu.gzDps) &&
        rollAnglePid.kp > 0.0f && pitchAnglePid.kp > 0.0f &&
        rollRatePid.kp > 0.0f && pitchRatePid.kp > 0.0f &&
        !(DISABLE_MOTORS_ON_CRITICAL_BATTERY && batteryCritical);
    int16_t descentThrottleUs = THROTTLE_LOW_CUTOFF_US;
    if (!healthy || !failsafeDescent.advance(nowUs, THROTTLE_LOW_CUTOFF_US,
                                             descentThrottleUs)) {
      failsafeDescent.cancel();
      armed = false;
      resetDrehmControllerStates();
      resetPidTraceCapture();
    } else {
      receiverCommand.roll = FAILSAFE_ROLL;
      receiverCommand.pitch = FAILSAFE_PITCH;
      receiverCommand.yaw = FAILSAFE_YAW;
      receiverCommand.throttle = descentThrottleUs;
      startupSafetyCleared = false;
      return;  // Link recovery cannot hand control back during descent.
    }
  }
  if (failsafeActive) {
    startupSafetyCleared = false;
    receiverCommand.roll = FAILSAFE_ROLL;
    receiverCommand.pitch = FAILSAFE_PITCH;
    receiverCommand.yaw = FAILSAFE_YAW;
    receiverCommand.throttle = FAILSAFE_THROTTLE_US;
  } else if (radioOk && imuOk && imuBiasValid && !imuCalibrating &&
              receiverCommand.throttle <= STARTUP_SAFE_THROTTLE_US &&
             fabsf(imu.rollDeg) <= ARM_MAX_TILT_DEG &&
             fabsf(imu.pitchDeg) <= ARM_MAX_TILT_DEG) {
    startupSafetyCleared = true;
  }
}

/* Explicit arming.

   Before this, motor output became legal the moment the preconditions held:
   power on, link up, stick down, frame level, and the props were live with no
   pilot action of any kind. Now the pilot has to ask for it.

   The transmitter already sends its joystick push switch in button bit 0 and
   this sketch already stored it without ever reading it, so arming costs no
   packet bytes and leaves the descriptor hash untouched.

   Both arming and disarming require the throttle to be down. That is the part
   that matters: a stray or bouncing button read at flying throttle cannot cut
   the motors mid-air. Losing the link, or anything else that clears the startup
   gate, disarms unconditionally - and since the edge detector updates on every
   call, a button still held down through a failsafe cannot re-arm by itself. */
void updateArming() {
  if (failsafeDescent.active()) return;
#if !REQUIRE_EXPLICIT_ARM
  // Pre-arming behaviour: no pilot action, output just follows the startup gate.
  armed = !failsafeActive && startupSafetyCleared;
#else
  const bool buttonNow = (receiverCommand.buttons & CONTROL_BUTTON_ARM_MASK) != 0;
  const bool buttonRising = buttonNow && !armButtonPrev;
  armButtonPrev = buttonNow;

  if (failsafeActive) {
    // Do not clear armButtonReleaseRequired using stale receiverCommand state.
    // A real released control frame must arrive after link recovery.
    armed = false;
    return;
  }

  // STOP deliberately consumes both a button already down and one whose packet
  // arrives later in this loop. Only release, then a subsequent rising edge,
  // can make arming eligible again.
  if (armButtonReleaseRequired) {
    if (!buttonNow) {
      armButtonReleaseRequired = false;
    }
    armed = false;
    return;
  }

  if (!startupSafetyCleared) {
    armed = false;
    return;
  }

  if (!buttonRising) {
    return;
  }
  if (receiverCommand.throttle > STARTUP_SAFE_THROTTLE_US) {
    return;
  }

  const uint32_t nowMs = millis();
  if (nowMs - lastArmToggleMs < ARM_TOGGLE_LOCKOUT_MS) {
    return;
  }
  // Recheck at the actual arm edge: the startup gate may have latched while
  // level, then the pilot may have picked up or tilted the aircraft.
  if (!armed && (!radioOk || !imuOk || !imuBiasValid || imuCalibrating ||
                 link.calibrationRunning() || !controlTimerOk ||
                 !isfinite(imu.rollDeg) || !isfinite(imu.pitchDeg) ||
                 fabsf(imu.rollDeg) > ARM_MAX_TILT_DEG ||
                 fabsf(imu.pitchDeg) > ARM_MAX_TILT_DEG)) return;
  lastArmToggleMs = nowMs;
  armed = !armed;
#endif
}

// -----------------------------
// MPU6050 IMU
// -----------------------------
bool mpuWriteRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(mpu6050I2cAddress);
  Wire.write(reg);
  Wire.write(value);
  return (Wire.endTransmission() == 0);
}

bool mpuReadBytes(uint8_t reg, uint8_t *buffer, size_t length) {
  Wire.beginTransmission(mpu6050I2cAddress);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  const size_t received =
      Wire.requestFrom(static_cast<int>(mpu6050I2cAddress), static_cast<int>(length));
  if (received != length) {
    return false;
  }

  for (size_t i = 0; i < length; ++i) {
    buffer[i] = Wire.read();
  }

  return true;
}

bool mpuWhoAmIIsPlausible(uint8_t whoAmI) {
  for (uint8_t i = 0; i < sizeof(MPU6050_WHO_AM_I_VALUES); ++i) {
    if (whoAmI == MPU6050_WHO_AM_I_VALUES[i]) return true;
  }
  return false;
}

bool initIMU() {
  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  Wire.begin();
  // 1 MHz Fast-mode Plus request is intentionally over the common MPU6050
  // module spec; it is an experimental dRehm-profile requirement, not proven
  // by this build. Hardware must demonstrate clean bus edges/NACK-free reads.
  Wire.setClock(1000000);

  // Probe both legal AD0 addresses so a module's hardware strap needs no edit.
  uint8_t whoAmI = 0;
  mpu6050I2cAddress = MPU6050_I2C_ADDR_LOW;
  bool detected = mpuReadBytes(MPU6050_REG_WHO_AM_I, &whoAmI, 1) &&
                  mpuWhoAmIIsPlausible(whoAmI);
  if (!detected) {
    mpu6050I2cAddress = MPU6050_I2C_ADDR_HIGH;
    detected = mpuReadBytes(MPU6050_REG_WHO_AM_I, &whoAmI, 1) &&
               mpuWhoAmIIsPlausible(whoAmI);
  }
  if (!detected) return false;

  if (!mpuWriteRegister(MPU6050_REG_PWR_MGMT_1, MPU6050_CMD_DEVICE_RESET)) return false;
  delay(100);
  // Selecting the X-gyro PLL also clears sleep and gives the sensor a stable clock.
  if (!mpuWriteRegister(MPU6050_REG_PWR_MGMT_1, MPU6050_WAKE_PLL_XGYRO)) return false;
  delay(100);

  if (!mpuWriteRegister(MPU6050_REG_CONFIG, MPU6050_DLPF_OFF) ||
      !mpuWriteRegister(MPU6050_REG_SMPLRT_DIV, MPU6050_SAMPLE_RATE_DIV) ||
      !mpuWriteRegister(MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCEL_RANGE_4G) ||
      !mpuWriteRegister(MPU6050_REG_GYRO_CONFIG, MPU6050_GYRO_RANGE_2000DPS) ||
      !mpuWriteRegister(MPU6050_REG_INT_PIN_CFG, 0x00u) ||
      !mpuWriteRegister(MPU6050_REG_INT_ENABLE, 0x01u)) {
    return false;
  }

  uint8_t accelRange = 0;
  uint8_t gyroRange = 0;
  uint8_t interruptEnable = 0;
  uint8_t interruptPinConfig = 0;
  if (!mpuReadBytes(MPU6050_REG_ACCEL_CONFIG, &accelRange, 1) ||
      !mpuReadBytes(MPU6050_REG_GYRO_CONFIG, &gyroRange, 1) ||
      !mpuReadBytes(MPU6050_REG_INT_ENABLE, &interruptEnable, 1) ||
      !mpuReadBytes(MPU6050_REG_INT_PIN_CFG, &interruptPinConfig, 1)) {
    return false;
  }
  return (accelRange & MPU6050_FS_SEL_MASK) == MPU6050_ACCEL_RANGE_4G &&
         (gyroRange & MPU6050_FS_SEL_MASK) == MPU6050_GYRO_RANGE_2000DPS &&
         interruptEnable == 0x01u && interruptPinConfig == 0x00u;
}

bool readRawMPU6050(int16_t &gx, int16_t &gy, int16_t &gz,
                    int16_t &ax, int16_t &ay, int16_t &az) {
  uint8_t status = 0;
  if (!mpuReadBytes(MPU6050_REG_INT_STATUS, &status, 1)) return false;
  imuSampleFresh = (status & 0x01u) != 0u;
  if (!imuSampleFresh) return true;
  // Detect an in-flight reset or configuration loss without repeatedly
  // reconfiguring the sensor or changing the established bus settings.
  static uint16_t configCounter = 0;
  if (++configCounter >= 200u) {
    configCounter = 0;
    static uint8_t configPhase = 0;
    const uint8_t registers[] = {MPU6050_REG_PWR_MGMT_1, MPU6050_REG_CONFIG,
        MPU6050_REG_SMPLRT_DIV, MPU6050_REG_ACCEL_CONFIG, MPU6050_REG_GYRO_CONFIG,
        MPU6050_REG_INT_ENABLE, MPU6050_REG_INT_PIN_CFG};
    const uint8_t expected[] = {MPU6050_WAKE_PLL_XGYRO, MPU6050_DLPF_OFF,
        MPU6050_SAMPLE_RATE_DIV, MPU6050_ACCEL_RANGE_4G, MPU6050_GYRO_RANGE_2000DPS,
        0x01u, 0x00u};
    uint8_t actual = 0;
    if (!mpuReadBytes(registers[configPhase], &actual, 1)) return false;
    if (configPhase == 3 || configPhase == 4) actual &= MPU6050_FS_SEL_MASK;
    if (actual != expected[configPhase]) return false;
    configPhase = (configPhase + 1u) % 7u;
  }
  // MPU6050 burst order is accel XYZ, temperature, then gyro XYZ; all big-endian.
  uint8_t buffer[14];
  if (!mpuReadBytes(MPU6050_REG_ACCEL_XOUT_H, buffer, sizeof(buffer))) return false;
  ax = static_cast<int16_t>((static_cast<uint16_t>(buffer[0]) << 8) | buffer[1]);
  ay = static_cast<int16_t>((static_cast<uint16_t>(buffer[2]) << 8) | buffer[3]);
  az = static_cast<int16_t>((static_cast<uint16_t>(buffer[4]) << 8) | buffer[5]);
  gx = static_cast<int16_t>((static_cast<uint16_t>(buffer[8]) << 8) | buffer[9]);
  gy = static_cast<int16_t>((static_cast<uint16_t>(buffer[10]) << 8) | buffer[11]);
  gz = static_cast<int16_t>((static_cast<uint16_t>(buffer[12]) << 8) | buffer[13]);
  return true;
}

void startImuCalibration() {
  imuCalibrationCapture = ImuCalibrationCapture{};
  imuCalibrationMotionDetected = false;
  imuCalibrationSamples = 0;
  imuCalibrationGyroSpanDps = 0.0f;
  imuCalibrationCapture.startMs = millis();
  imuCalibrationCapture.lastSampleMs = millis() - IMU_CALIBRATION_SAMPLE_PERIOD_MS;
  imuCalibrating = true;
  imuBiasValid = false;
  startupSafetyCleared = false;
  armed = false;
  resetDrehmControllerStates();
  link.state(true, false, "IMU calibration started; hold level and still");
}

void updateCalibrationRequest() {
  const bool buttonNow = (receiverCommand.buttons & CONTROL_BUTTON_CAL_MASK) != 0;
  const bool buttonRising = buttonNow && !calButtonPrev;
  calButtonPrev = buttonNow;

  if (failsafeActive) {
    calButtonReleaseRequired = true;
    return;
  }
  if (calButtonReleaseRequired) {
    if (!buttonNow) calButtonReleaseRequired = false;
    return;
  }
  if (!buttonRising || imuCalibrating || link.calibrationRunning()) return;

  if (!radioOk || !imuInitialized || !imuOk || !controlTimerOk || armed ||
      receiverCommand.throttle > STARTUP_SAFE_THROTTLE_US ||
      (receiverCommand.buttons & CONTROL_BUTTON_ARM_MASK) != 0) {
    link.error("IMU_CAL_REJECTED");
    return;
  }
  startImuCalibration();
}

void captureImuCalibrationSample(int16_t rawGx, int16_t rawGy, int16_t rawGz,
                                 int16_t rawAx, int16_t rawAy, int16_t rawAz) {
  if (!imuCalibrating) return;
  const uint32_t nowMs = millis();
  if (nowMs - imuCalibrationCapture.lastSampleMs < IMU_CALIBRATION_SAMPLE_PERIOD_MS) return;
  imuCalibrationCapture.lastSampleMs = nowMs;

  float gyroDps[3] = {
      static_cast<float>(rawGx) / IMU_GYRO_LSB_PER_DPS,
      static_cast<float>(rawGy) / IMU_GYRO_LSB_PER_DPS,
      static_cast<float>(rawGz) / IMU_GYRO_LSB_PER_DPS,
  };
  // Calibration-only median: one isolated peak cannot poison the motion
  // extrema or gyro bias. Wait for a full window, including at startup.
  for (uint8_t axis = 0; axis < 3; ++axis) {
    imuCalibrationCapture.gyroHistoryDps[axis][imuCalibrationCapture.gyroHistoryIndex] = gyroDps[axis];
  }
  imuCalibrationCapture.gyroHistoryIndex = (imuCalibrationCapture.gyroHistoryIndex + 1u) % 3u;
  if (imuCalibrationCapture.gyroHistoryCount < 3u) ++imuCalibrationCapture.gyroHistoryCount;
  if (imuCalibrationCapture.gyroHistoryCount < 3u) return;
  for (uint8_t axis = 0; axis < 3; ++axis) {
    const float a = imuCalibrationCapture.gyroHistoryDps[axis][0];
    const float b = imuCalibrationCapture.gyroHistoryDps[axis][1];
    const float c = imuCalibrationCapture.gyroHistoryDps[axis][2];
    gyroDps[axis] = fmaxf(fminf(a, b), fminf(fmaxf(a, b), c));
  }
  imuCalibrationCapture.axSum += static_cast<float>(rawAx) / IMU_ACC_LSB_PER_G;
  imuCalibrationCapture.aySum += static_cast<float>(rawAy) / IMU_ACC_LSB_PER_G;
  imuCalibrationCapture.azSum += static_cast<float>(rawAz) / IMU_ACC_LSB_PER_G;
  imuCalibrationCapture.gxSum += gyroDps[0];
  imuCalibrationCapture.gySum += gyroDps[1];
  imuCalibrationCapture.gzSum += gyroDps[2];
  if (imuCalibrationCapture.motionWindowCount < IMU_CALIBRATION_MOTION_WINDOW_SAMPLES)
    ++imuCalibrationCapture.motionWindowCount;
  for (uint8_t axis = 0; axis < 3; ++axis) {
    imuCalibrationCapture.motionSumDps[axis] += gyroDps[axis] -
        imuCalibrationCapture.motionWindowDps[axis][imuCalibrationCapture.motionWindowIndex];
    imuCalibrationCapture.motionWindowDps[axis][imuCalibrationCapture.motionWindowIndex] = gyroDps[axis];
    if (imuCalibrationCapture.motionWindowCount == IMU_CALIBRATION_MOTION_WINDOW_SAMPLES) {
      const float mean = imuCalibrationCapture.motionSumDps[axis] /
                         IMU_CALIBRATION_MOTION_WINDOW_SAMPLES;
      if (mean < imuCalibrationCapture.motionMinDps[axis]) imuCalibrationCapture.motionMinDps[axis] = mean;
      if (mean > imuCalibrationCapture.motionMaxDps[axis]) imuCalibrationCapture.motionMaxDps[axis] = mean;
      if (fabsf(mean) > IMU_CALIBRATION_MOTION_LIMIT_DPS ||
          imuCalibrationCapture.motionMaxDps[axis] - imuCalibrationCapture.motionMinDps[axis] >
              IMU_CALIBRATION_MOTION_LIMIT_DPS)
        imuCalibrationMotionDetected = true;
    }
    if (gyroDps[axis] < imuCalibrationCapture.gyroMinDps[axis])
      imuCalibrationCapture.gyroMinDps[axis] = gyroDps[axis];
    if (gyroDps[axis] > imuCalibrationCapture.gyroMaxDps[axis])
      imuCalibrationCapture.gyroMaxDps[axis] = gyroDps[axis];
  }
  imuCalibrationCapture.motionWindowIndex = (imuCalibrationCapture.motionWindowIndex + 1u) %
                                           IMU_CALIBRATION_MOTION_WINDOW_SAMPLES;
  ++imuCalibrationCapture.samples;
}

void printImuCalibrationDiagnostics(const char *result) {
  const uint32_t samples = imuCalibrationCapture.samples;
  const float divisor = samples > 0u ? static_cast<float>(samples) : 1.0f;
  const float meanAx = imuCalibrationCapture.axSum / divisor;
  const float meanAy = imuCalibrationCapture.aySum / divisor;
  const float meanAz = imuCalibrationCapture.azSum / divisor;
  const float gravityG = sqrtf(meanAx * meanAx + meanAy * meanAy + meanAz * meanAz);
  float gyroSpan[3] = {0.0f, 0.0f, 0.0f};
  if (samples > 0u) {
    for (uint8_t axis = 0; axis < 3; ++axis) {
      gyroSpan[axis] = imuCalibrationCapture.gyroMaxDps[axis] -
                       imuCalibrationCapture.gyroMinDps[axis];
    }
  }

  usbProtocol.print(F("CAL_DIAG,result=")); usbProtocol.print(result);
  usbProtocol.print(F(",samples=")); usbProtocol.print(samples);
  usbProtocol.print(F(",elapsed_ms=")); usbProtocol.print(millis() - imuCalibrationCapture.startMs);
  usbProtocol.print(F(",ax_g=")); usbProtocol.print(meanAx, 5);
  usbProtocol.print(F(",ay_g=")); usbProtocol.print(meanAy, 5);
  usbProtocol.print(F(",az_g=")); usbProtocol.print(meanAz, 5);
  usbProtocol.print(F(",gravity_g=")); usbProtocol.print(gravityG, 5);
  usbProtocol.print(F(",gx_span_dps=")); usbProtocol.print(gyroSpan[0], 4);
  usbProtocol.print(F(",gy_span_dps=")); usbProtocol.print(gyroSpan[1], 4);
  usbProtocol.print(F(",gz_span_dps=")); usbProtocol.print(gyroSpan[2], 4);
  usbProtocol.print(F(",motion=")); usbProtocol.print(imuCalibrationMotionDetected ? 1 : 0);
  usbProtocol.print(F(",motion_filter=mean25_v2"));
  for (uint8_t axis = 0; axis < 3; ++axis) {
    usbProtocol.print(F(",motion_axis")); usbProtocol.print(axis);
    usbProtocol.print(F("_min_dps="));
    usbProtocol.print(imuCalibrationCapture.motionWindowCount == IMU_CALIBRATION_MOTION_WINDOW_SAMPLES ?
                 imuCalibrationCapture.motionMinDps[axis] : 0.0f, 4);
    usbProtocol.print(F(",motion_axis")); usbProtocol.print(axis);
    usbProtocol.print(F("_max_dps="));
    usbProtocol.print(imuCalibrationCapture.motionWindowCount == IMU_CALIBRATION_MOTION_WINDOW_SAMPLES ?
                 imuCalibrationCapture.motionMaxDps[axis] : 0.0f, 4);
  }
  usbProtocol.print(F(",sample_fresh=")); usbProtocol.print(imuSampleFresh ? 1 : 0);
  usbProtocol.print(F(",imu_initialized=")); usbProtocol.print(imuInitialized ? 1 : 0);
  usbProtocol.print(F(",imu_ok=")); usbProtocol.print(imuOk ? 1 : 0);
  usbProtocol.print(F(",radio_ok=")); usbProtocol.print(radioOk ? 1 : 0);
  usbProtocol.print(F(",failsafe=")); usbProtocol.print(failsafeActive ? 1 : 0);
  usbProtocol.print(F(",timer_ok=")); usbProtocol.print(controlTimerOk ? 1 : 0);
  usbProtocol.print(F(",throttle_us=")); usbProtocol.println(receiverCommand.throttle);
}

void updateImuCalibration() {
  if (!imuCalibrating) return;
  if (failsafeActive || armed || !imuOk ||
      receiverCommand.throttle > STARTUP_SAFE_THROTTLE_US) {
    imuCalibrating = false;
    printImuCalibrationDiagnostics("ABORTED");
    link.error("IMU_CAL_ABORTED");
    return;
  }
  if (millis() - imuCalibrationCapture.startMs < IMU_CALIBRATION_DURATION_MS) return;

  imuCalibrating = false;
  imuCalibrationSamples = imuCalibrationCapture.samples;
  if (imuCalibrationSamples < IMU_CALIBRATION_MIN_SAMPLES) {
    printImuCalibrationDiagnostics("TOO_FEW_SAMPLES");
    link.error("IMU_CAL_TOO_FEW_SAMPLES");
    return;
  }

  imuCalibrationGyroSpanDps = 0.0f;
  for (uint8_t axis = 0; axis < 3; ++axis) {
    const float span = imuCalibrationCapture.gyroMaxDps[axis] -
                       imuCalibrationCapture.gyroMinDps[axis];
    if (span > imuCalibrationGyroSpanDps) imuCalibrationGyroSpanDps = span;
  }
  // Instantaneous span is diagnostic only; capture already checked averaged motion.
  if (imuCalibrationMotionDetected) {
    printImuCalibrationDiagnostics("MOTION");
    link.error("IMU_CAL_MOTION");
    return;
  }

  const float meanAx = imuCalibrationCapture.axSum / imuCalibrationSamples;
  const float meanAy = imuCalibrationCapture.aySum / imuCalibrationSamples;
  const float meanAz = imuCalibrationCapture.azSum / imuCalibrationSamples;
  const float magnitude2 = meanAx * meanAx + meanAy * meanAy + meanAz * meanAz;
  // Accept a stable, upright MPU reading down to 0.80 g; the stationary
  // calibration below restores the level Z reading to 1 g. Keep the separate
  // 0.80-1.20 g magnitude gate so gross sensor errors still block arming.
  if (fabsf(meanAx) > 0.26f || fabsf(meanAy) > 0.26f || meanAz < 0.80f ||
      magnitude2 < 0.64f || magnitude2 > 1.44f) {
    printImuCalibrationDiagnostics("NOT_LEVEL");
    link.error("IMU_CAL_NOT_LEVEL");
    return;
  }

  imuBias.axG = meanAx;
  imuBias.ayG = meanAy;
  imuBias.azG = meanAz - IMU_LEVEL_SENSOR_Z_G;
  imuBias.gxDps = imuCalibrationCapture.gxSum / imuCalibrationSamples;
  imuBias.gyDps = imuCalibrationCapture.gySum / imuCalibrationSamples;
  imuBias.gzDps = imuCalibrationCapture.gzSum / imuCalibrationSamples;
  imu = ImuState{};
  resetDrehmControllerStates();
  imuBiasValid = true;
  printImuCalibrationDiagnostics("OK");
  link.state(true, false, "IMU calibration complete; release then press arm");
}

/* 6DOF Madgwick gradient step, adapted to the frozen local body frame. The
   official dRehm sketch's signed sensor call is NOT copied: sample gyro signs
   are the established local positive roll/pitch/yaw conventions. */
void updateMadgwick6Dof(float gxDps, float gyDps, float gzDps,
                        float axG, float ayG, float azG, float dtSeconds) {
  float q0 = imu.q0, q1 = imu.q1, q2 = imu.q2, q3 = imu.q3;
  const float gx = gxDps * 0.01745329251994329577f;
  const float gy = gyDps * 0.01745329251994329577f;
  const float gz = gzDps * 0.01745329251994329577f;
  float qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  float qDot1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
  float qDot2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
  float qDot3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);
  const float aNorm2 = axG * axG + ayG * ayG + azG * azG;
  if (aNorm2 > 1.0e-8f) {
    const float invA = 1.0f / sqrtf(aNorm2);
    const float ax = axG * invA, ay = ayG * invA, az = azG * invA;
    const float f1 = 2.0f * (q1*q3 - q0*q2) - ax;
    const float f2 = 2.0f * (q0*q1 + q2*q3) - ay;
    const float f3 = 2.0f * (0.5f - q1*q1 - q2*q2) - az;
    float s0 = -2.0f*q2*f1 + 2.0f*q1*f2;
    float s1 =  2.0f*q3*f1 + 2.0f*q0*f2 - 4.0f*q1*f3;
    float s2 = -2.0f*q0*f1 + 2.0f*q3*f2 - 4.0f*q2*f3;
    float s3 =  2.0f*q1*f1 + 2.0f*q2*f2;
    const float sNorm2 = s0*s0 + s1*s1 + s2*s2 + s3*s3;
    if (sNorm2 > 1.0e-12f) {
      const float invS = 1.0f / sqrtf(sNorm2);
      qDot0 -= DREHM_MADGWICK_BETA * s0 * invS;
      qDot1 -= DREHM_MADGWICK_BETA * s1 * invS;
      qDot2 -= DREHM_MADGWICK_BETA * s2 * invS;
      qDot3 -= DREHM_MADGWICK_BETA * s3 * invS;
    }
  }
  q0 += qDot0 * dtSeconds; q1 += qDot1 * dtSeconds;
  q2 += qDot2 * dtSeconds; q3 += qDot3 * dtSeconds;
  const float qNorm2 = q0*q0 + q1*q1 + q2*q2 + q3*q3;
  if (qNorm2 > 1.0e-12f) {
    const float invQ = 1.0f / sqrtf(qNorm2);
    imu.q0 = q0 * invQ; imu.q1 = q1 * invQ;
    imu.q2 = q2 * invQ; imu.q3 = q3 * invQ;
  }
  imu.rawRollDeg = atan2f(2.0f*(imu.q0*imu.q1 + imu.q2*imu.q3),
                          1.0f - 2.0f*(imu.q1*imu.q1 + imu.q2*imu.q2)) * RAD_TO_DEG_LOCAL;
  const float sinPitch = clampFloat(2.0f*(imu.q0*imu.q2 - imu.q3*imu.q1), -1.0f, 1.0f);
  imu.rawPitchDeg = asinf(sinPitch) * RAD_TO_DEG_LOCAL;
  imu.yawDeg = atan2f(2.0f*(imu.q0*imu.q3 + imu.q1*imu.q2),
                       1.0f - 2.0f*(imu.q2*imu.q2 + imu.q3*imu.q3)) * RAD_TO_DEG_LOCAL;
}

bool readIMU(float dtSeconds) {
  static float estimatorElapsedSeconds = 0.0f;
  estimatorElapsedSeconds += dtSeconds;
  int16_t rawGx, rawGy, rawGz, rawAx, rawAy, rawAz;
  if (!readRawMPU6050(rawGx, rawGy, rawGz, rawAx, rawAy, rawAz)) {
    estimatorElapsedSeconds = 0.0f;
    return false;
  }
  if (!imuSampleFresh) {
    const bool withinFreshnessWindow =
        lastFreshImuUs != 0 && micros() - lastFreshImuUs <= 3000UL;
    if (!withinFreshnessWindow) estimatorElapsedSeconds = 0.0f;
    return withinFreshnessWindow;
  }
  const float estimatorDtSeconds = estimatorElapsedSeconds;
  estimatorElapsedSeconds = 0.0f;
  lastFreshImuUs = micros();
  captureImuCalibrationSample(rawGx, rawGy, rawGz, rawAx, rawAy, rawAz);

  const float sensorAx = (static_cast<float>(rawAx) / IMU_ACC_LSB_PER_G) - imuBias.axG;
  const float sensorAy = (static_cast<float>(rawAy) / IMU_ACC_LSB_PER_G) - imuBias.ayG;
  const float sensorAz = (static_cast<float>(rawAz) / IMU_ACC_LSB_PER_G) - imuBias.azG;
  const float sensorGx = (static_cast<float>(rawGx) / IMU_GYRO_LSB_PER_DPS) - imuBias.gxDps;
  const float sensorGy = (static_cast<float>(rawGy) / IMU_GYRO_LSB_PER_DPS) - imuBias.gyDps;
  const float sensorGz = (static_cast<float>(rawGz) / IMU_GYRO_LSB_PER_DPS) - imuBias.gzDps;

  // Mounted sensor: +X left, +Y back, +Z up. Both streams use the same
  // right-handed rotation into the established front/left/up body frame.
  // Positive body +Z yaw is nose-left; yaw output raises M1/M4 (CW).
  PidLinkImuSample sample;
  sample.accel[0] = -sensorAy;
  sample.accel[1] = sensorAx;
  sample.accel[2] = sensorAz;
  sample.gyro[0] = -sensorGy;
  sample.gyro[1] = sensorGx;
  sample.gyro[2] = sensorGz;
  sample.roll = atan2f(sample.accel[1], sample.accel[2]) * RAD_TO_DEG_LOCAL;
  sample.pitch = atan2f(-sample.accel[0],
                        sqrtf(sample.accel[1] * sample.accel[1] +
                              sample.accel[2] * sample.accel[2])) * RAD_TO_DEG_LOCAL;
  sample.hasAttitude = true;

  // Keep an uncorrected copy for CAL,START, then apply the persisted residuals
  // to the values that actually enter the filters and the control loops.
  baseImuSample = sample;
  haveBaseImuSample = true;
  link.applyCalibration(sample);

  imu.axG += DREHM_BACCEL * (sample.accel[0] - imu.axG);
  imu.ayG += DREHM_BACCEL * (sample.accel[1] - imu.ayG);
  imu.azG += DREHM_BACCEL * (sample.accel[2] - imu.azG);
  imu.gxDps += DREHM_BGYRO * (sample.gyro[0] - imu.gxDps);
  imu.gyDps += DREHM_BGYRO * (sample.gyro[1] - imu.gyDps);
  imu.gzDps += DREHM_BGYRO * (sample.gyro[2] - imu.gzDps);
  updateMadgwick6Dof(imu.gxDps, imu.gyDps, imu.gzDps,
                      imu.axG, imu.ayG, imu.azG, estimatorDtSeconds);

  // link.applyCalibration(sample) already applied the saved residual once to
  // the accelerometer data feeding this attitude estimate. Subtracting the
  // saved roll/pitch again here used to double the level correction and could
  // manufacture a small false lean after CAL,SAVE.
  imu.rollDeg = imu.rawRollDeg;
  imu.pitchDeg = imu.rawPitchDeg;

  return true;
}

// -----------------------------
// Motor and debug PWM output
// -----------------------------
void writeAllMotorsMinimum() {
  motorOutputUs[0] = MOTOR_MIN_US;
  motorOutputUs[1] = MOTOR_MIN_US;
  motorOutputUs[2] = MOTOR_MIN_US;
  motorOutputUs[3] = MOTOR_MIN_US;
}

#if ESC_PROTOCOL_DSHOT300 || ESC_PROTOCOL_DSHOT600
void initDShotOutputs() {
  // initMotorOutputs() has already selected GPIO output and driven every pin
  // LOW. Keep GPIO selected; the old incomplete FlexPWM/DMA path is not used.
  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
  const uint32_t before = ARM_DWT_CYCCNT;
  asm volatile("nop\n nop\n nop\n nop" ::: "memory");
  dshotOutputHealthy = F_CPU_ACTUAL >= 300000000UL && ARM_DWT_CYCCNT != before;
}
#endif

void initMotorOutputs() {
  // Pins idle LOW until the first duty cycle is written, so the ESCs see no
  // signal rather than a stray pulse while the rest of setup() runs.
  pinMode(MOTOR1_CMD_PIN, OUTPUT);
  pinMode(MOTOR2_CMD_PIN, OUTPUT);
  pinMode(MOTOR3_CMD_PIN, OUTPUT);
  pinMode(MOTOR4_CMD_PIN, OUTPUT);
  digitalWriteFast(MOTOR1_CMD_PIN, LOW);
  digitalWriteFast(MOTOR2_CMD_PIN, LOW);
  digitalWriteFast(MOTOR3_CMD_PIN, LOW);
  digitalWriteFast(MOTOR4_CMD_PIN, LOW);

  writeAllMotorsMinimum();

  if (MOTOR_PROTOCOL == MOTOR_PROTOCOL_STANDARD_PWM) {
    analogWriteResolution(MOTOR_PWM_RESOLUTION_BITS);
    analogWriteFrequency(MOTOR1_CMD_PIN, MOTOR_PWM_FREQUENCY_HZ);
    analogWriteFrequency(MOTOR2_CMD_PIN, MOTOR_PWM_FREQUENCY_HZ);
    analogWriteFrequency(MOTOR3_CMD_PIN, MOTOR_PWM_FREQUENCY_HZ);
    analogWriteFrequency(MOTOR4_CMD_PIN, MOTOR_PWM_FREQUENCY_HZ);
  } else if (MOTOR_PROTOCOL_IS_DSHOT) {
#if ESC_PROTOCOL_DSHOT300 || ESC_PROTOCOL_DSHOT600
    initDShotOutputs();
#endif
  }

  // setup() calls writeMotorOutputs() next, which pushes MOTOR_MIN_US out.
}

void initDebugPWMOutputs() {
#if ENABLE_DEBUG_PWM_OUTPUTS
  debugThrottlePwm.attach(RC_PWM_THROTTLE_DBG_PIN, THROTTLE_MIN_US, THROTTLE_MAX_US);
  debugYawPwm.attach(RC_PWM_YAW_DBG_PIN, THROTTLE_MIN_US, THROTTLE_MAX_US);
  debugPitchPwm.attach(RC_PWM_PITCH_DBG_PIN, THROTTLE_MIN_US, THROTTLE_MAX_US);
  debugRollPwm.attach(RC_PWM_ROLL_DBG_PIN, THROTTLE_MIN_US, THROTTLE_MAX_US);

  debugThrottlePwm.writeMicroseconds(THROTTLE_MIN_US);
  debugYawPwm.writeMicroseconds(THROTTLE_NEUTRAL_US);
  debugPitchPwm.writeMicroseconds(THROTTLE_NEUTRAL_US);
  debugRollPwm.writeMicroseconds(THROTTLE_NEUTRAL_US);
#endif
}

bool motorOutputAllowed() {
#if ESC_PROTOCOL_DSHOT300 || ESC_PROTOCOL_DSHOT600
  if (!dshotOutputHealthy) return false;
#endif
  if (!imuOk || !imuBiasValid || imuCalibrating || link.calibrationRunning() ||
      !controlTimerOk || (!failsafeDescent.active() &&
                          (!radioOk || failsafeActive || !startupSafetyCleared))) {
    return false;
  }
  if (!armed) {
    return false;
  }
  if (DISABLE_MOTORS_ON_CRITICAL_BATTERY && batteryCritical) {
    return false;
  }
  return true;
}

uint16_t makeDShotPacket(uint16_t value, bool requestTelemetry) {
  value = static_cast<uint16_t>(clampInt32(value, DSHOT_STOP_VALUE, DSHOT_MAX_THROTTLE));
  uint16_t packet = static_cast<uint16_t>((value << 1) | (requestTelemetry ? 1u : 0u));
  uint16_t checksumData = packet;
  uint8_t checksum = 0;
  for (uint8_t i = 0; i < 3; ++i) {
    checksum ^= static_cast<uint8_t>(checksumData & 0x0Fu);
    checksumData >>= 4;
  }
  return static_cast<uint16_t>((packet << 4) | (checksum & 0x0Fu));
}

uint16_t motorMicrosecondsToDShot(int16_t pulseUs) {
  return static_cast<uint16_t>(mapLinearClamped(pulseUs,
                                                 MOTOR_IDLE_US,
                                                 MOTOR_MAX_US,
                                                 DSHOT_MIN_THROTTLE,
                                                 DSHOT_MAX_THROTTLE));
}

#if ESC_PROTOCOL_DSHOT300 || ESC_PROTOCOL_DSHOT600
// All deadline arithmetic is unsigned elapsed time, including DWT rollover.
// A stopped counter or unexpectedly late edge aborts instead of spinning
// indefinitely or continuing a malformed motor packet.
FASTRUN bool waitDShotDeadline(uint32_t start, uint32_t deadline, uint32_t maxLate) {
  for (uint32_t poll = 0; poll < DSHOT_MAX_WAIT_POLLS; ++poll) {
    const uint32_t elapsed = ARM_DWT_CYCCNT - start;
    if (elapsed >= deadline) return elapsed - deadline <= maxLate;
  }
  return false;
}

FASTRUN bool writeDShotPackets(const uint16_t packets[DSHOT_MOTOR_COUNT]) {
  if (!dshotOutputHealthy || F_CPU_ACTUAL < 300000000UL) return false;

  // Prepare masks before masking interrupts. M3/M4 share GPIO6 and switch
  // together; GPIO7 (M1) and GPIO9 (M2) follow the same fixed write ordering.
  uint32_t shortClear7[DSHOT_FRAME_BITS];
  uint32_t shortClear9[DSHOT_FRAME_BITS];
  uint32_t shortClear6[DSHOT_FRAME_BITS];
  for (uint8_t bit = 0; bit < DSHOT_FRAME_BITS; ++bit) {
    const uint16_t mask = static_cast<uint16_t>(0x8000u >> bit);
    shortClear7[bit] = (packets[0] & mask) ? 0u : CORE_PIN8_BITMASK;
    shortClear9[bit] = (packets[1] & mask) ? 0u : CORE_PIN4_BITMASK;
    shortClear6[bit] = ((packets[2] & mask) ? 0u : CORE_PIN22_BITMASK) |
                       ((packets[3] & mask) ? 0u : CORE_PIN23_BITMASK);
  }

  const uint32_t bitCycles = static_cast<uint32_t>(
      (static_cast<uint64_t>(F_CPU_ACTUAL) * DSHOT_BIT_DURATION_NS) / 1000000000ULL);
  const uint32_t shortCycles = static_cast<uint32_t>(
      (static_cast<uint64_t>(F_CPU_ACTUAL) * DSHOT_SHORT_PULSE_NS) / 1000000000ULL);
  const uint32_t longCycles = static_cast<uint32_t>(
      (static_cast<uint64_t>(F_CPU_ACTUAL) * DSHOT_LONG_PULSE_NS) / 1000000000ULL);
  const uint32_t maxLateCycles = static_cast<uint32_t>(
      (static_cast<uint64_t>(F_CPU_ACTUAL) * DSHOT_MAX_EDGE_LATENESS_NS) / 1000000000ULL);

  // Gap occurs before the new critical section, with the caller's IRQ state
  // intact. Outputs are already LOW, including for consecutive STOP packets.
  GPIO7_DR_CLEAR = CORE_PIN8_BITMASK;
  GPIO9_DR_CLEAR = CORE_PIN4_BITMASK;
  GPIO6_DR_CLEAR = CORE_PIN22_BITMASK | CORE_PIN23_BITMASK;
  delayMicroseconds(DSHOT_LOW_GAP_US);

  uint32_t savedPrimask;
  asm volatile("mrs %0, primask\n cpsid i" : "=r"(savedPrimask) :: "memory");
  const uint32_t frameStart = ARM_DWT_CYCCNT;
  bool complete = true;
  for (uint8_t bit = 0; bit < DSHOT_FRAME_BITS; ++bit) {
    const uint32_t bitStart = static_cast<uint32_t>(bit) * bitCycles;
    if (!waitDShotDeadline(frameStart, bitStart, maxLateCycles)) { complete = false; break; }
    GPIO7_DR_SET = CORE_PIN8_BITMASK;
    GPIO9_DR_SET = CORE_PIN4_BITMASK;
    GPIO6_DR_SET = CORE_PIN22_BITMASK | CORE_PIN23_BITMASK;
    if (!waitDShotDeadline(frameStart, bitStart + shortCycles, maxLateCycles)) { complete = false; break; }
    GPIO7_DR_CLEAR = shortClear7[bit];
    GPIO9_DR_CLEAR = shortClear9[bit];
    GPIO6_DR_CLEAR = shortClear6[bit];
    if (!waitDShotDeadline(frameStart, bitStart + longCycles, maxLateCycles)) { complete = false; break; }
    GPIO7_DR_CLEAR = CORE_PIN8_BITMASK;
    GPIO9_DR_CLEAR = CORE_PIN4_BITMASK;
    GPIO6_DR_CLEAR = CORE_PIN22_BITMASK | CORE_PIN23_BITMASK;
  }
  if (complete)
    complete = waitDShotDeadline(frameStart, DSHOT_FRAME_BITS * bitCycles, maxLateCycles);

  // Always leave all four outputs LOW and restore the caller's PRIMASK, even
  // on a deadline fault. No asynchronous peripheral can reassert an output.
  GPIO7_DR_CLEAR = CORE_PIN8_BITMASK;
  GPIO9_DR_CLEAR = CORE_PIN4_BITMASK;
  GPIO6_DR_CLEAR = CORE_PIN22_BITMASK | CORE_PIN23_BITMASK;
  asm volatile("msr primask, %0" :: "r"(savedPrimask) : "memory");
  return complete;
}
#endif

void writeDShotMotorOutputs() {
#if ESC_PROTOCOL_DSHOT300 || ESC_PROTOCOL_DSHOT600
  const bool sendStop =
      !motorOutputAllowed() || receiverCommand.throttle <= THROTTLE_LOW_CUTOFF_US;
  uint16_t packets[4];
  for (uint8_t i = 0; i < 4; ++i) {
    const uint16_t dshotValue =
        sendStop ? DSHOT_STOP_VALUE : motorMicrosecondsToDShot(motorOutputUs[i]);
    packets[i] = makeDShotPacket(dshotValue, DSHOT_REQUEST_TELEMETRY);
  }
  if (!writeDShotPackets(packets)) {
    dshotOutputHealthy = false;  // Latch until reboot; never retry a bad waveform.
    ++dshotOutputFaultCount;
    failsafeDescent.cancel();
    armed = false;
    startupSafetyCleared = false;
    armButtonReleaseRequired = true;
    writeAllMotorsMinimum();
    resetDrehmControllerStates();
  }
#endif
}

void writeOneShot125Pulses() {
  uint16_t pulseUs[4];
  for (uint8_t i = 0; i < 4; ++i) {
    pulseUs[i] = static_cast<uint16_t>(mapLinearClamped(motorOutputUs[i], MOTOR_MIN_US, MOTOR_MAX_US, 125, 250));
  }

  digitalWriteFast(MOTOR1_CMD_PIN, HIGH);
  digitalWriteFast(MOTOR2_CMD_PIN, HIGH);
  digitalWriteFast(MOTOR3_CMD_PIN, HIGH);
  digitalWriteFast(MOTOR4_CMD_PIN, HIGH);

  const uint32_t startUs = micros();
  bool high[4] = {true, true, true, true};
  while (high[0] || high[1] || high[2] || high[3]) {
    const uint32_t elapsed = micros() - startUs;
    if (high[0] && elapsed >= pulseUs[0]) {
      digitalWriteFast(MOTOR1_CMD_PIN, LOW);
      high[0] = false;
    }
    if (high[1] && elapsed >= pulseUs[1]) {
      digitalWriteFast(MOTOR2_CMD_PIN, LOW);
      high[1] = false;
    }
    if (high[2] && elapsed >= pulseUs[2]) {
      digitalWriteFast(MOTOR3_CMD_PIN, LOW);
      high[2] = false;
    }
    if (high[3] && elapsed >= pulseUs[3]) {
      digitalWriteFast(MOTOR4_CMD_PIN, LOW);
      high[3] = false;
    }
  }
}

void writeMotorPwmMicroseconds(uint8_t pin, int16_t pulseUs) {
  const int32_t duty = lroundf((static_cast<float>(pulseUs) / MOTOR_PWM_FRAME_US) *
                               static_cast<float>(MOTOR_PWM_MAX_COUNT));
  analogWrite(pin, static_cast<int>(clampInt32(duty, 0, MOTOR_PWM_MAX_COUNT)));
}

void writeMotorOutputs() {
  for (uint8_t i = 0; i < 4; ++i) {
    motorOutputUs[i] = static_cast<int16_t>(clampInt32(motorOutputUs[i], MOTOR_MIN_US, MOTOR_MAX_US));
  }

  switch (MOTOR_PROTOCOL) {
    case MOTOR_PROTOCOL_STANDARD_PWM:
      writeMotorPwmMicroseconds(MOTOR1_CMD_PIN, motorOutputUs[0]);
      writeMotorPwmMicroseconds(MOTOR2_CMD_PIN, motorOutputUs[1]);
      writeMotorPwmMicroseconds(MOTOR3_CMD_PIN, motorOutputUs[2]);
      writeMotorPwmMicroseconds(MOTOR4_CMD_PIN, motorOutputUs[3]);
      break;
    case MOTOR_PROTOCOL_ONESHOT125:
      writeOneShot125Pulses();
      break;
    case MOTOR_PROTOCOL_DSHOT300:
    case MOTOR_PROTOCOL_DSHOT600:
      writeDShotMotorOutputs();
      break;
  }
}

void updateDebugPWMOutputs() {
#if ENABLE_DEBUG_PWM_OUTPUTS
  debugThrottlePwm.writeMicroseconds(receiverCommand.throttle);
  debugYawPwm.writeMicroseconds(commandToRcPulseUs(receiverCommand.yaw));
  debugPitchPwm.writeMicroseconds(commandToRcPulseUs(receiverCommand.pitch));
  debugRollPwm.writeMicroseconds(commandToRcPulseUs(receiverCommand.roll));
#endif
}

// -----------------------------
// Flight control
// -----------------------------
void resetPid(PidState &state) {
  state.integrator = 0.0f;
  state.previousMeasurement = 0.0f;
  state.filteredDerivativeTerm = 0.0f;
  state.pTerm = 0.0f;
  state.iTerm = 0.0f;
  state.dTerm = 0.0f;
  state.measurementInitialized = false;
  state.previousOutput = 0.0f;
}

// Reset every dRehm accumulator, error derivative and angle smoothing state at
// each explicit STOP, remote STOP and radio-failsafe disarm boundary.
void resetDrehmControllerStates() {
  resetPid(rollPidState);
  resetPid(pitchPidState);
  resetPid(yawPidState);
  resetPid(rollAnglePidState);
  resetPid(pitchAnglePidState);
  rollPidOutputUs = pitchPidOutputUs = yawPidOutputUs = 0.0f;
}

// derivativeLimit is in the controller's own OUTPUT units, so the rate loops
// pass microseconds and the angle loops pass deg/s. Sharing one microsecond
// constant across both would silently clamp the outer loop in the wrong unit.
// Deliberately not a defaulted parameter: the Arduino builder copies default
// arguments into the prototypes it generates for a .ino, which then clash with
// the definition here.
float updatePid(const PidGains &gains,
                PidState &state,
                float setpoint,
                float measurement,
                float dtSeconds,
                float integratorLimit,
                float outputLimit,
                float derivativeLimit) {
  const float error = setpoint - measurement;

  // Differentiate the measured rate instead of the error. This avoids a large
  // derivative kick whenever the pilot moves a stick and changes the setpoint.
  float derivativeTerm = 0.0f;
  if (state.measurementInitialized && dtSeconds > 0.0f) {
    const float measurementDerivative = (measurement - state.previousMeasurement) / dtSeconds;
    const float rawDerivativeTerm = -gains.kd * measurementDerivative;
    const float derivativeBlend = lowPassBlend(dtSeconds, PID_D_FILTER_TAU_S);
    state.filteredDerivativeTerm +=
        derivativeBlend * (rawDerivativeTerm - state.filteredDerivativeTerm);
    state.filteredDerivativeTerm =
        clampFloat(state.filteredDerivativeTerm, -derivativeLimit, derivativeLimit);
    derivativeTerm = state.filteredDerivativeTerm;
  } else {
    state.filteredDerivativeTerm = 0.0f;
    state.measurementInitialized = true;
  }
  state.previousMeasurement = measurement;

  const float proportionalTerm = gains.kp * error;
  const float candidateIntegrator =
      clampFloat(state.integrator + error * gains.ki * dtSeconds, -integratorLimit, integratorLimit);
  const float candidateOutput = proportionalTerm + candidateIntegrator + derivativeTerm;

  // Integrate across the full usable rate range, but stop charging farther into
  // output saturation. Opposite-sign error is still allowed to unwind the I term.
  const bool pushesHighSaturation = candidateOutput > outputLimit && error > 0.0f;
  const bool pushesLowSaturation = candidateOutput < -outputLimit && error < 0.0f;
  if (!pushesHighSaturation && !pushesLowSaturation) {
    state.integrator = candidateIntegrator;
  }

  state.pTerm = proportionalTerm;
  state.iTerm = state.integrator;
  state.dTerm = derivativeTerm;
  const float output = state.pTerm + state.iTerm + state.dTerm;
  return clampFloat(output, -outputLimit, outputLimit);
}

// dRehmFlight controlANGLE2-compatible stages. The local mixer consumes virtual
// motor microseconds, so normalized dRehm terms are multiplied by 1000 only at
// this boundary. TPA intentionally is NOT in this path: official dRehm does
// not attenuate P/D by throttle.
constexpr float DREHM_ANGLE_KL = 30.0f;
constexpr float DREHM_ANGLE_RATE_LIMIT_DPS = 240.0f;
constexpr float DREHM_ANGLE_SMOOTH_B = 0.90f;
constexpr float DREHM_NORMALIZED_TO_VIRTUAL_US = 1000.0f;
constexpr float DREHM_RATE_OUTPUT_LIMIT_US = 250.0f;

float updateDrehmAngle(const PidGains &gains, PidState &state,
                       float setpointDeg, float angleDeg, float dtSeconds) {
  const float error = setpointDeg - angleDeg;
  state.integrator = clampFloat(state.integrator + error * dtSeconds,
                                -DREHM_INTEGRATOR_RAW_LIMIT, DREHM_INTEGRATOR_RAW_LIMIT);
  float dMeasurement = 0.0f;
  if (state.measurementInitialized && dtSeconds > 0.0f) {
    dMeasurement = (angleDeg - state.previousMeasurement) / dtSeconds;
  } else {
    state.measurementInitialized = true;
  }
  state.previousMeasurement = angleDeg;
  state.pTerm = gains.kp * error * DREHM_ANGLE_KL;
  state.iTerm = gains.ki * state.integrator * DREHM_ANGLE_KL;
  // Official source leaves this term commented. It is deliberately active and
  // tunable here, zero by default; measurement derivative avoids stick kick.
  state.dTerm = -gains.kd * dMeasurement * DREHM_ANGLE_KL;
  const float raw = clampFloat(state.pTerm + state.iTerm + state.dTerm,
                               -DREHM_ANGLE_RATE_LIMIT_DPS, DREHM_ANGLE_RATE_LIMIT_DPS);
  state.previousOutput = (1.0f - DREHM_ANGLE_SMOOTH_B) * state.previousOutput +
                         DREHM_ANGLE_SMOOTH_B * raw;
  return state.previousOutput;
}

float updateDrehmRate(const PidGains &gains, PidState &state,
                      float setpointDps, float measurementDps, float dtSeconds) {
  const float error = setpointDps - measurementDps;
  state.integrator = clampFloat(state.integrator + error * dtSeconds,
                                -DREHM_INTEGRATOR_RAW_LIMIT, DREHM_INTEGRATOR_RAW_LIMIT);
  float errorDerivative = 0.0f;
  if (state.measurementInitialized && dtSeconds > 0.0f) {
    errorDerivative = (error - state.previousMeasurement) / dtSeconds;
  } else {
    state.measurementInitialized = true;
  }
  state.previousMeasurement = error;
  // Preserve these *actual dRehm terms* in the local mixer unit for trace.
  state.pTerm = 0.01f * gains.kp * error * DREHM_NORMALIZED_TO_VIRTUAL_US;
  state.iTerm = 0.01f * gains.ki * state.integrator * DREHM_NORMALIZED_TO_VIRTUAL_US;
  state.dTerm = 0.01f * gains.kd * errorDerivative * DREHM_NORMALIZED_TO_VIRTUAL_US;
  return clampFloat(state.pTerm + state.iTerm + state.dTerm,
                    -DREHM_RATE_OUTPUT_LIMIT_US, DREHM_RATE_OUTPUT_LIMIT_US);
}

/* Bounded 50% cubic expo for roll/pitch only. It is odd and monotonic on
   [-1, +1], with y(0)=0 and y(+/-1)=+/-1. */
float shapeRollPitchStickNormalized(int16_t stickCommand) {
  const float x = clampFloat(static_cast<float>(stickCommand) /
                                 static_cast<float>(COMMAND_MAX),
                             -1.0f, 1.0f);
  return (1.0f - ROLL_PITCH_CUBIC_EXPO) * x +
         ROLL_PITCH_CUBIC_EXPO * x * x * x;
}

/* Betaflight's stick-to-rate curve. normalizedStick is -1..+1; the result is
   deg/s. Reproduced rather than approximated so the aircraft answers the sticks
   the way the Betaflight build did. */
float normalizedStickToRateDps(float normalizedStick, float maxRateDps) {
  float s = clampFloat(normalizedStick, -1.0f, 1.0f);

  if (BF_RC_EXPO > 0.0f) {
    const float a = fabsf(s);
    s = s * a * a * a * BF_RC_EXPO + s * (1.0f - BF_RC_EXPO);
  }

  float rate = 200.0f * BF_RC_RATE * s;

  const float super = clampFloat(BF_SUPER_RATE, 0.0f, BF_SUPER_RATE_MAX);
  if (super > 0.0f) {
    // 1/(1-x) runs away as the stick approaches full deflection, so the factor
    // is bounded before it is used rather than after it has already produced
    // an infinity.
    const float denom = 1.0f - fabsf(s) * super;
    rate *= 1.0f / ((denom < 0.01f) ? 0.01f : denom);
  }

  // Scale the curve to the configured ceiling instead of clipping against it.
  // Clipping would flatten everything past the stick position that first hits
  // the ceiling - at a 160 deg/s yaw limit that is barely half of stick travel,
  // so the outer half would do nothing. Scaling keeps the soft centre and the
  // full range usable at any ceiling, and at 667 the factor is 1.0, which is
  // the Betaflight curve untouched.
  const float peak = 200.0f * BF_RC_RATE / (1.0f - super);
  if (peak > 0.0f) rate *= maxRateDps / peak;

  return clampFloat(rate, -maxRateDps, maxRateDps);
}

// Existing linear raw-stick entry point; yaw deliberately continues to use it.
float stickToRateDps(int16_t stickCommand, float maxRateDps) {
  return normalizedStickToRateDps(static_cast<float>(stickCommand) /
                                      static_cast<float>(COMMAND_MAX),
                                  maxRateDps);
}

float rollPitchStickToRateDps(int16_t stickCommand, float maxRateDps) {
  return normalizedStickToRateDps(shapeRollPitchStickNormalized(stickCommand), maxRateDps);
}

/* Throttle PID Attenuation. Returns the factor applied to P and D above the
   breakpoint - never to I, which is what holds the aircraft level and must not
   fade out with throttle. */
float throttlePidAttenuation(int16_t throttleUs) {
  if (throttleUs <= TPA_BREAKPOINT_US) return 1.0f;
  const float span = static_cast<float>(MOTOR_MAX_US - TPA_BREAKPOINT_US);
  if (span <= 0.0f) return 1.0f;
  const float above = static_cast<float>(throttleUs - TPA_BREAKPOINT_US) / span;
  return clampFloat(1.0f - TPA_RATE * clampFloat(above, 0.0f, 1.0f),
                    1.0f - TPA_RATE, 1.0f);
}

/* A scaled copy, so the tuner keeps showing and setting the real gains rather
   than whatever TPA happened to leave behind at the throttle of the moment. */
PidGains attenuatedGains(const PidGains &base, float factor) {
  PidGains out = base;
  out.kp = base.kp * factor;
  out.kd = base.kd * factor;
  return out;
}

float slewToward(float current, float target, float maxStep) {
  if (target > current + maxStep) return current + maxStep;
  if (target < current - maxStep) return current - maxStep;
  return target;
}

void updateFlightControl(float dtSeconds) {
  if (!motorOutputAllowed() || receiverCommand.throttle <= THROTTLE_LOW_CUTOFF_US) {
    // With the motors safely stopped there is nothing to jerk, so preload the
    // requested trim before takeoff instead of slowly changing it after arming.
    activeLevelTrimRollDeg = levelTrimDeg.kp;
    activeLevelTrimPitchDeg = levelTrimDeg.ki;
    rollPidOutputUs = 0.0f;
    pitchPidOutputUs = 0.0f;
    yawPidOutputUs = 0.0f;
    resetPid(rollPidState);
    resetPid(pitchPidState);
    resetPid(yawPidState);
    // The angle integrators are cleared on the ground too. Sitting on uneven
    // legs is a large but meaningless angle error, and letting it accumulate
    // would hand the aircraft a wound-up correction at the instant of liftoff.
    resetPid(rollAnglePidState);
    resetPid(pitchAnglePidState);
    lastDesiredRollRateDps = 0.0f;
    lastDesiredPitchRateDps = 0.0f;
    lastDesiredYawRateDps = 0.0f;
    lastCommandedRollDeg = 0.0f;
    lastCommandedPitchDeg = 0.0f;
    writeAllMotorsMinimum();
    return;
  }

  float desiredRollRateDps;
  float desiredPitchRateDps;

  const float trimStepDeg = LEVEL_TRIM_SLEW_RATE_DEG_PER_S * dtSeconds;
  activeLevelTrimRollDeg = slewToward(activeLevelTrimRollDeg, levelTrimDeg.kp, trimStepDeg);
  activeLevelTrimPitchDeg = slewToward(activeLevelTrimPitchDeg, levelTrimDeg.ki, trimStepDeg);

  if (angleModeEnabled || failsafeDescent.active()) {
    // Outer loop: the stick asks for an ANGLE and this loop works out what
    // rotation rate would get there. Centred sticks therefore mean "be level",
    // not merely "stop rotating", which is what lets it hold a lean out.
    // Hover trim shifts the angle target itself. Positive pitch is nose-down;
    // positive roll is right-side-down in this firmware's body convention.
    const float commandedRollDeg = clampFloat(
        shapeRollPitchStickNormalized(receiverCommand.roll) * MAX_LEVEL_ANGLE_DEG +
            activeLevelTrimRollDeg,
        -MAX_LEVEL_ANGLE_DEG, MAX_LEVEL_ANGLE_DEG);
    const float commandedPitchDeg = clampFloat(
        shapeRollPitchStickNormalized(receiverCommand.pitch) * MAX_LEVEL_ANGLE_DEG +
            activeLevelTrimPitchDeg,
        -MAX_LEVEL_ANGLE_DEG, MAX_LEVEL_ANGLE_DEG);

    desiredRollRateDps = updateDrehmAngle(rollAnglePid, rollAnglePidState,
                                           commandedRollDeg, imu.rollDeg, dtSeconds);
    desiredPitchRateDps = updateDrehmAngle(pitchAnglePid, pitchAnglePidState,
                                            commandedPitchDeg, imu.pitchDeg, dtSeconds);

    desiredRollRateDps = clampFloat(desiredRollRateDps, -MAX_ROLL_RATE_DPS, MAX_ROLL_RATE_DPS);
    desiredPitchRateDps = clampFloat(desiredPitchRateDps, -MAX_PITCH_RATE_DPS, MAX_PITCH_RATE_DPS);

    lastCommandedRollDeg = commandedRollDeg;
    lastCommandedPitchDeg = commandedPitchDeg;
  } else {
    lastCommandedRollDeg = 0.0f;
    lastCommandedPitchDeg = 0.0f;
    desiredRollRateDps = rollPitchStickToRateDps(receiverCommand.roll, MAX_ROLL_RATE_DPS);
    desiredPitchRateDps = rollPitchStickToRateDps(receiverCommand.pitch, MAX_PITCH_RATE_DPS);
  }

  // Yaw stays a rate command in both modes: there is no absolute heading
  // reference to level against, only a rate to hold.
  const float desiredYawRateDps = stickToRateDps(receiverCommand.yaw, MAX_YAW_RATE_DPS);

  lastDesiredRollRateDps = desiredRollRateDps;
  lastDesiredPitchRateDps = desiredPitchRateDps;
  lastDesiredYawRateDps = desiredYawRateDps;

  // Official dRehm has no throttle PID attenuation. Retain local throttle cap
  // and differential mixer, but bypass the baseline TPA controller path.
  rollPidOutputUs = updateDrehmRate(rollRatePid, rollPidState,
                                    desiredRollRateDps, imu.gxDps, dtSeconds);
  pitchPidOutputUs = updateDrehmRate(pitchRatePid, pitchPidState,
                                     desiredPitchRateDps, imu.gyDps, dtSeconds);
  yawPidOutputUs = updateDrehmRate(yawRatePid, yawPidState,
                                   desiredYawRateDps, imu.gzDps, dtSeconds);
}

void mixMotors() {
  if (!motorOutputAllowed() || receiverCommand.throttle <= THROTTLE_LOW_CUTOFF_US) {
    writeAllMotorsMinimum();
    lastMixSaturated = false;
    return;
  }

  // Per-motor correction first, with no throttle in it yet. Seeding the extents
  // at zero keeps the collective from ever being pushed below MOTOR_IDLE_US.
  float mixUs[4];
  float mixMinUs = 0.0f;
  float mixMaxUs = 0.0f;
  for (uint8_t i = 0; i < 4; ++i) {
    mixUs[i] = MOTOR_MIX[i].rollSign * rollPidOutputUs +
               MOTOR_MIX[i].pitchSign * pitchPidOutputUs +
               MOTOR_MIX[i].yawSign * yawPidOutputUs;
    if (mixUs[i] < mixMinUs) {
      mixMinUs = mixUs[i];
    }
    if (mixUs[i] > mixMaxUs) {
      mixMaxUs = mixUs[i];
    }
  }

  // Attitude control comes from the DIFFERENCE between motors, never from any
  // one motor's absolute value. So when the corrections span more than the
  // usable output range, shrink them all by the same factor instead of letting
  // individual motors hit the rails. Clamping per motor silently deletes the
  // differential and drops motors to zero while the opposite pair spins up.
  const float availableRangeUs = static_cast<float>(MOTOR_MAX_US - MOTOR_IDLE_US);
  const float requestedRangeUs = mixMaxUs - mixMinUs;
  bool saturated = requestedRangeUs > availableRangeUs;
  if (requestedRangeUs > availableRangeUs) {
    const float scale = availableRangeUs / requestedRangeUs;
    for (uint8_t i = 0; i < 4; ++i) {
      mixUs[i] *= scale;
    }
    mixMinUs *= scale;
    mixMaxUs *= scale;
  }

  // Slide the collective so every motor lands inside [idle, max] with its
  // correction intact. Trades a little throttle accuracy for never dropping a
  // motor mid-correction. Throttle at the bottom stop is still a hard cut,
  // handled by the THROTTLE_LOW_CUTOFF_US check above.
  // Throttle Limit SCALE, as configured in Betaflight. Applied to the pilot's
  // throttle before the mix, so the attitude corrections keep their full
  // authority - a limiter that also shrank the corrections would make the
  // aircraft progressively less controllable the harder you pushed it.
  const float limitedThrottleUs =
      static_cast<float>(MOTOR_MIN_US) +
      (static_cast<float>(receiverCommand.throttle) - static_cast<float>(MOTOR_MIN_US)) *
          THROTTLE_LIMIT_SCALE;

  const float lowestAllowedBaseUs = static_cast<float>(MOTOR_IDLE_US) - mixMinUs;
  const float highestAllowedBaseUs = static_cast<float>(MOTOR_MAX_US) - mixMaxUs;
  const float baseThrottleUs = clampFloat(limitedThrottleUs,
                                          lowestAllowedBaseUs,
                                          highestAllowedBaseUs);
  if (fabsf(baseThrottleUs - limitedThrottleUs) > 0.5f) saturated = true;

  for (uint8_t i = 0; i < 4; ++i) {
    motorOutputUs[i] = static_cast<int16_t>(
        clampInt32(lroundf(baseThrottleUs + mixUs[i]),
                   MOTOR_MIN_US, MOTOR_MAX_US));
    if (motorOutputUs[i] <= MOTOR_IDLE_US || motorOutputUs[i] >= MOTOR_MAX_US) {
      saturated = true;
    }
  }
  lastMixSaturated = saturated;
}

// -----------------------------
// Battery and LEDs
// -----------------------------
void updateBatteryMonitor() {
  const uint16_t raw = analogRead(VBAT_SENSE_PIN);
  const float adcVoltage = (static_cast<float>(raw) / ADC_MAX_VALUE) * ADC_REFERENCE_VOLTS;
  batteryVoltage = adcVoltage * BATTERY_DIVIDER_RATIO;

  batteryCritical = batteryVoltage > 0.5f && batteryVoltage <= BATTERY_CRITICAL_VOLTS;
  batteryLow = batteryVoltage > 0.5f && batteryVoltage <= BATTERY_LOW_VOLTS;
}

void updateStatusLEDs() {
  const uint32_t nowMs = millis();

  // LINK LED meaning, in priority order. The rule worth memorising: solid, or
  // solid with a wink, is the only state in which a motor can turn.
  //   fast flash (5 Hz)   - commanded IMU bias capture running; hold level/still
  //   off                 - nRF24L01 did not initialize on SPI
  //   slow flash (1 Hz)   - radio up but no control packets, failsafe active
  //   double blip         - linked, IMU calibration required; press left stick
  //   brief blip (0.5 Hz) - calibrated, DISARMED; press right stick
  //   solid               - armed, motors live
  if (imuCalibrating) {
    digitalWrite(LINK_LED_CTRL_PIN,
                 ((nowMs / IMU_CALIBRATION_LED_PERIOD_MS) % 2UL) ? HIGH : LOW);
  } else if (!radioOk) {
    digitalWrite(LINK_LED_CTRL_PIN, LOW);
  } else if (failsafeActive) {
    digitalWrite(LINK_LED_CTRL_PIN, ((nowMs / 500UL) % 2UL) ? HIGH : LOW);
  } else if (!imuBiasValid) {
    const uint32_t phaseMs = nowMs % 2000UL;
    digitalWrite(LINK_LED_CTRL_PIN,
                 (phaseMs < 120UL || (phaseMs >= 240UL && phaseMs < 360UL)) ? HIGH : LOW);
  } else if (!armed) {
    digitalWrite(LINK_LED_CTRL_PIN, ((nowMs % 2000UL) < 120UL) ? HIGH : LOW);
  } else {
    digitalWrite(LINK_LED_CTRL_PIN, HIGH);
  }

  // The final board has no battery LED. Battery state remains available in
  // telemetry; link LED behavior stays dedicated to link/arming safety.
}

// -----------------------------
// usbProtocol debug
// -----------------------------
void printDebugStatus() {
#if DEBUG_SERIAL
  usbProtocol.print(F("RF="));
  usbProtocol.print(radioOk ? F("OK") : F("FAIL"));
  usbProtocol.print(F(" IMU="));
  usbProtocol.print(imuOk ? F("OK") : F("FAIL"));
  usbProtocol.print(F(" TIMER="));
  usbProtocol.print(controlTimerOk ? F("OK") : F("FAIL"));
  usbProtocol.print(F(" FS="));
  usbProtocol.print(failsafeActive ? F("YES") : F("NO"));
  usbProtocol.print(F(" SAFE="));
  usbProtocol.print(startupSafetyCleared ? F("CLR") : F("WAIT"));
  usbProtocol.print(F(" ARM="));
  usbProtocol.print(armed ? F("ARMED") : F("safe"));
  if (imuCalibrationMotionDetected) {
    usbProtocol.print(F(" CAL=MOVED"));
  }
  usbProtocol.print(F(" seq="));
  usbProtocol.print(receiverCommand.sequenceNumber);
  usbProtocol.print(F(" R/P/Y/T="));
  usbProtocol.print(receiverCommand.roll);
  usbProtocol.print('/');
  usbProtocol.print(receiverCommand.pitch);
  usbProtocol.print('/');
  usbProtocol.print(receiverCommand.yaw);
  usbProtocol.print('/');
  usbProtocol.print(receiverCommand.throttle);
  usbProtocol.print(F(" Vbat="));
  usbProtocol.print(batteryVoltage, 2);
  usbProtocol.print(F(" gyro="));
  usbProtocol.print(imu.gxDps, 1);
  usbProtocol.print('/');
  usbProtocol.print(imu.gyDps, 1);
  usbProtocol.print('/');
  usbProtocol.print(imu.gzDps, 1);
  usbProtocol.print(F(" mot="));
  usbProtocol.print(motorOutputUs[0]);
  usbProtocol.print('/');
  usbProtocol.print(motorOutputUs[1]);
  usbProtocol.print('/');
  usbProtocol.print(motorOutputUs[2]);
  usbProtocol.print('/');
  usbProtocol.print(motorOutputUs[3]);
  usbProtocol.print(F(" pkt="));
  usbProtocol.print(validPacketCount);
  usbProtocol.print(F(" bad="));
  usbProtocol.print(invalidPacketCount);
  usbProtocol.print(F(" drop="));
  usbProtocol.print(droppedSequenceCount);
  usbProtocol.print(F(" ctrl_us="));
  usbProtocol.print(lastControlExecutionUs);
  usbProtocol.print('/');
  usbProtocol.print(longestControlExecutionUs);
  usbProtocol.print(F(" skip="));
  usbProtocol.print(skippedControlTicks);
  usbProtocol.print(F(" overrun="));
  usbProtocol.print(controlExecutionOverruns);
  usbProtocol.print(F(" interval_us="));
  usbProtocol.print(lastControlStartIntervalUs);
  usbProtocol.print('/');
  usbProtocol.print(longestControlStartIntervalUs);
  usbProtocol.print(F(" deadline_miss="));
  usbProtocol.print(controlDeadlineMisses);
  usbProtocol.print(F(" late_start="));
  usbProtocol.print(controlLateStarts);
  usbProtocol.print(F(" fg_us="));
  usbProtocol.print(lastForegroundExecutionUs);
  usbProtocol.print('/');
  usbProtocol.print(longestForegroundExecutionUs);
  usbProtocol.print(F(" rf_us="));
  usbProtocol.print(radioServiceUs);
  usbProtocol.print('/');
  usbProtocol.print(longestRadioServiceUs);
  usbProtocol.print(F(" imu_us="));
  usbProtocol.print(lastImuReadUs);
  usbProtocol.print('/');
  usbProtocol.print(longestImuReadUs);
  usbProtocol.print(F(" rf_descent="));
  usbProtocol.print(failsafeDescent.active() ? F("YES") : F("NO"));
  usbProtocol.print(F(" PIDr="));
  usbProtocol.print(rollPidState.pTerm, 1);
  usbProtocol.print('/');
  usbProtocol.print(rollPidState.iTerm, 1);
  usbProtocol.print('/');
  usbProtocol.print(rollPidState.dTerm, 1);
  usbProtocol.print(F(" rate="));
  usbProtocol.println(radioDataRateLabel());
#endif
}

// -----------------------------
// Arduino lifecycle
// -----------------------------
void setup() {
  // Opens usbProtocol and hashes the descriptor. Must run before anything can answer
  // SYS,DESCRIBE or announce an identity over the radio.
  initPidLink();
#if DEBUG_SERIAL
  delay(300);
#endif

  pinMode(LINK_LED_CTRL_PIN, OUTPUT);
  digitalWrite(LINK_LED_CTRL_PIN, LOW);

  analogReadResolution(ADC_BITS);
  analogReadAveraging(8);

  initMotorOutputs();
  initDebugPWMOutputs();
  writeMotorOutputs();

  radioOk = initRadio();
  imuInitialized = initIMU();
  imuOk = imuInitialized;

  // Bias capture is manual: wait for a fresh left-stick press from the TX.

  updateBatteryMonitor();
  updateFailsafe(micros());
  updateStatusLEDs();

  scheduledControlTicks = 0;
  processedControlTicks = 0;
  controlTimerOk = controlTimer.begin(controlTimerISR, CONTROL_LOOP_PERIOD_US);
  queueTelemetryAckPayload();
  link.state(linkIsStopped(), false,
             imuOk ? "IMU calibration required: press left stick" : "IMU unavailable");
  lastBatteryUpdateMs = millis();
  lastDebugPwmUpdateMs = millis();
  lastLedUpdateMs = millis();
  lastSerialDebugMs = millis();
}

void loop() {
  const uint32_t foregroundStartUs = micros();
  usbProtocol.pump();
  const uint32_t nowMs = millis();

  // Non-blocking: only consumes USB bytes already buffered, and advances a
  // running IMU calibration.
  link.poll();

  const uint32_t radioStartUs = micros();
  updateReceiver();
  radioServiceUs = micros() - radioStartUs;
  if (radioServiceUs > longestRadioServiceUs) longestRadioServiceUs = radioServiceUs;
  // updateReceiver() timestamps a valid packet with micros(). Sample the
  // timeout clock afterward so unsigned subtraction cannot wrap when a packet
  // arrived later than a stale loop-start timestamp.
  updateFailsafe(micros());
  updateCalibrationRequest();
  updateArming();

  uint32_t availableControlTicks;
  noInterrupts();
  availableControlTicks = scheduledControlTicks;
  interrupts();

  const uint32_t elapsedControlTicks = availableControlTicks - processedControlTicks;
  if (controlTimerOk && elapsedControlTicks > 0) {
    processedControlTicks = availableControlTicks;
    if (elapsedControlTicks > 1) {
      skippedControlTicks += elapsedControlTicks - 1;
      controlDeadlineMisses += elapsedControlTicks - 1;
    }

    const uint32_t controlStartUs = micros();
    const uint32_t controlElapsedUs = previousControlStartUs != 0u ?
        controlStartUs - previousControlStartUs : CONTROL_LOOP_PERIOD_US;
    const float dtSeconds = static_cast<float>(controlElapsedUs) * 1.0e-6f;
    if (previousControlStartUs != 0) {
      lastControlStartIntervalUs = controlStartUs - previousControlStartUs;
      if (lastControlStartIntervalUs > longestControlStartIntervalUs)
        longestControlStartIntervalUs = lastControlStartIntervalUs;
      if (lastControlStartIntervalUs > CONTROL_LOOP_PERIOD_US * elapsedControlTicks)
        ++controlLateStarts;
    }
    previousControlStartUs = controlStartUs;
    if (imuInitialized) {
      const uint32_t imuReadStartUs = micros();
      const bool sampleOk = readIMU(dtSeconds);
      lastImuReadUs = micros() - imuReadStartUs;
      if (lastImuReadUs > longestImuReadUs) longestImuReadUs = lastImuReadUs;
      if (!sampleOk) {
        failsafeDescent.cancel();
        // Fail closed immediately. A later successful read may restore imuOk,
        // but startup safety must be cleared again at low throttle.
        imuOk = false;
        armed = false;
        armButtonReleaseRequired = true;
        haveBaseImuSample = false;
        startupSafetyCleared = false;
      } else if (!imuOk) {
        imuOk = true;
        startupSafetyCleared = false;
      }
    }

    updateFlightControl(dtSeconds);
    mixMotors();
    if ((processedControlTicks % TRACE_CONTROL_TICK_DIVIDER) == 0u) {
      updatePidTraceCapture(lastDesiredRollRateDps,
                            lastDesiredPitchRateDps,
                            lastDesiredYawRateDps);
    }
    writeMotorOutputs();

    if (++usbTelemetryCounter % usbTelemetryDivider == 0) emitUsbTelemetry();

    lastControlExecutionUs = micros() - controlStartUs;
    if (lastControlExecutionUs > longestControlExecutionUs) {
      longestControlExecutionUs = lastControlExecutionUs;
    }
    if (lastControlExecutionUs > CONTROL_LOOP_PERIOD_US) {
      ++controlExecutionOverruns;
    }
  }

  updateImuCalibration();

  if (nowMs - lastDebugPwmUpdateMs >= DEBUG_PWM_PERIOD_MS) {
    lastDebugPwmUpdateMs += DEBUG_PWM_PERIOD_MS;
    updateDebugPWMOutputs();
  }

  if (nowMs - lastBatteryUpdateMs >= BATTERY_UPDATE_INTERVAL_MS) {
    lastBatteryUpdateMs += BATTERY_UPDATE_INTERVAL_MS;
    updateBatteryMonitor();
  }

  if (nowMs - lastLedUpdateMs >= LED_UPDATE_INTERVAL_MS) {
    lastLedUpdateMs += LED_UPDATE_INTERVAL_MS;
    updateStatusLEDs();
  }

  if (nowMs - lastSerialDebugMs >= SERIAL_DEBUG_INTERVAL_MS) {
    lastSerialDebugMs += SERIAL_DEBUG_INTERVAL_MS;
    printDebugStatus();
  }
  lastForegroundExecutionUs = micros() - foregroundStartUs;
  if (lastForegroundExecutionUs > longestForegroundExecutionUs)
    longestForegroundExecutionUs = lastForegroundExecutionUs;
}
