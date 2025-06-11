# ESC-6STEP

Electronic speed control for BLDC motors with 6STEP algorithm - firmware.
Code for STSPIN32F0A microcontroller.

# Motor control
The 6-step algorithm was implemented using the Motor SDK. It allows tuning of algorithm parameters and includes the components necessary for motor control.

# Communication with ESC
The motor's speed can be controlled via a PWM signal or the DShot protocol.

UART communication was implemented based on McPacket and McTelemetryPacket, allowing the selection of the communication type (PWM, DShot, UART), motor speed control, and telemetry requests.

#
Full project: https://github.com/Michalek007/ESC
