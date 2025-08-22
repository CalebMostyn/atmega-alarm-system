# Smart Home Embedded System
Simple Arduino Door Lock and Smart Home System on the ATmega328P

## Description
Embedded system designed to do *relatively* arbitrary tasks similar to a real smart home system for a course on embedded systems.

### Alarm and Lock
The design features three push-buttons that must be pressed in a specific combination in order to lock and unlock the alarm. There is also a LCD controlled with TWI that reads out the current state of the alarm ("ARMED" or "DISARMED") as well as the state of the code input.

Inputting the correct code into the alarm while disarmed starts a 10 second countdown displayed on the LCD. Following the countdown, a "door lock" mechanism is engaged by rotating a stepper motor 90°. This will also turn on a red LED and turn off a green LED (another representation of the alarm state).

Once armed, the correct code must be input to revert the alarm back to disarmed, which does the opposite of arming it. If the incorrect code is input, the state changes to "ALARM", where the red LED will blink and a buzzer will sound.

### Help Button
The alarm also features a help push-button. When pressed, the alarm system will transmit a packet over USART to a seperate embedded system, alerting it of the help request. 

The seperate embedded system displays "HELP!" on its LCD upon receiving the packet, and pressing a push-button on it sends a status packet back to the alarm system over USART. Once the alarm system receives the correct status packet, it displays "OK" on the LCD for 2 seconds (instead of the normal alarm state), then returns to normal operation.
