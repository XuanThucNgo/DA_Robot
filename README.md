# Description
The robot receives velocity setpoints for both wheels from the ROS cmd_vel topic to control the motors. The STM32 microcontroller generates PWM signals to achieve the desired speed based on feedback from the encoders. A PID algorithm is implemented to precisely regulate motor speed. Simultaneously, the encoder pulse data is sent back to the embedded computer for feedback. Continuous data transmission and reception are handled via UART communication protocol combined with DMA.
## Flowchart system.
<picture>
  <img alt="Flowchart system" height="20%" width="20%" src="https://i.imgur.com/375uKg7.jpeg">
</picture>

## Flowchart PID algorithm.

<picture>
  <img alt="Flowchart PID algorithm" height="20%" width="20%" src="https://i.imgur.com/68ZZBFe.jpeg">
</picture>

## Module function
+ __Encoder__

Responsible for initializing and configuring a specific Timer for the Timer Encoder function. The Timer automatically counts encoder pulses, increasing or decreasing the count depending on the rotation direction determined by channels A and B.
It is used to measure the number of pulses within a defined sampling period (deltaT) for PID control, return the total pulse count when the encoder rotates (for functionality sent back to the embedded computer), and calculate the achieved velocity within the sampling period (deltaT).Reference [Encoder.md](README_Module/Encoder.md)
+ __PID__

Responsible for calculating the PID parameters for the motor based on the predefined setpoint. Both wheels have the same PID speed control parameters for the DC motors.Reference [PID.md](README_Module/PID.md)

+ __UART_Protocol__

Responsible for handling data transmission and reception using Interrupts and DMA. Reference [UART_Protocol.md](README_Module/UART_Protocol.md)


