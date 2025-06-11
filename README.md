# ESC-6STEP

Electronic speed control for BLDC motors with 6STEP algorithm - firmware.
Code for STSPIN32F0A microcontroller.

# Motor control
6-step algorithm was implemented using Motor SDK. It allows to tune parameters of algorithm and implements components neccessary for motor control.

# Communication with ESC
Motor's speed can be controlled via PWM signal or DShot protocol.

Implemented UART communication based on McPacket and McTelemetryPacket. It allows to set communication type (PWM, DSHOT, UART), control motor speed and request telemetry.

#
Full project: https://github.com/Michalek007/ESC
