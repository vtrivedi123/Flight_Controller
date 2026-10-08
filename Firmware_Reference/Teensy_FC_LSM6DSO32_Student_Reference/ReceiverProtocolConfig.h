#pragma once

// Select exactly one ESC signal protocol. Use only 0 or 1.
//
// OneShot125 is the default. The same airframe and ESCs were flown on a
// Betaflight OMNIBUSF4SD running ONESHOT125, so it is the protocol with
// evidence behind it here, not merely the safest guess. It also gives the
// 2 kHz control loop a 125-250 us pulse instead of a 1000-2000 us one. A
// standard PWM pulse cannot fit inside this build's 500 us control period.
//
// This 2 kHz experimental variant deliberately prohibits ESC_PROTOCOL_PWM.
#ifndef ESC_PROTOCOL_PWM
#define ESC_PROTOCOL_PWM 0
#endif

#ifndef ESC_PROTOCOL_ONESHOT125
#define ESC_PROTOCOL_ONESHOT125 1
#endif

#ifndef ESC_PROTOCOL_DSHOT300
#define ESC_PROTOCOL_DSHOT300 0
#endif

#ifndef ESC_PROTOCOL_DSHOT600
#define ESC_PROTOCOL_DSHOT600 0
#endif

#if ((ESC_PROTOCOL_PWM != 0) && (ESC_PROTOCOL_PWM != 1)) || \
    ((ESC_PROTOCOL_ONESHOT125 != 0) && (ESC_PROTOCOL_ONESHOT125 != 1)) || \
    ((ESC_PROTOCOL_DSHOT300 != 0) && (ESC_PROTOCOL_DSHOT300 != 1)) || \
    ((ESC_PROTOCOL_DSHOT600 != 0) && (ESC_PROTOCOL_DSHOT600 != 1))
#error "Every ESC_PROTOCOL_* switch must be either 0 or 1."
#endif

#if (ESC_PROTOCOL_PWM + ESC_PROTOCOL_ONESHOT125 + \
     ESC_PROTOCOL_DSHOT300 + ESC_PROTOCOL_DSHOT600) != 1
#error "Enable exactly one ESC protocol in ReceiverProtocolConfig.h."
#endif


