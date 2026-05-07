# dc-motor-pid-control
A closed-loop speed control system for DC motors using Arduino and PID algorithm with an real-time Optic Encoder.
​This project uses an Arduino Uno, L298N Motor Driver, and an FC-33 Optic Encoder.
​The goal of the project is to calculate and  maintain a constant target speed (Setpoint) regardless of load changes using pid controller.
​
Features:  
​Real-time Speed Measurement: Calculates speed in km/h using pulse counting with an encoder disk.
​PID Control: active feedback loop to maintain desired speed.
Schematics: wiring diagrams of arduino and components.
​
​Hardware Components:  
​Microcontroller: Arduino Uno
​Motor Driver: L298N Dual H-Bridge
​Actuator: 6V-12V DC Gear Motor
​Sensor: FC-33 Optic Encoder

​Repository Structure:  
​working-code/: Contains the working but hard to read code.
​clean-code/: The finalized, most readible code.
​schematics/: Wiring diagrams and circuit connection details.

Known İssues:  
Due to low pulse per rotation(spaces of disk) its hard to ensure an correct speed measurement in slower speeds and time period so if you have an disk
with an higher ppr please decrease the time reset period. With an one second time reset it is also takes longer for your motor to settle in your desired speed.
Due to overshooting(reading more pulses than reality) issues i added an simple if statement to ensure pulses readed within 10 milli seconds are not affecting the
actual pulse reading thats why in really high speeds(100 rotation per second) it will miss some pulses and will give you slower speed than it actually is.

​How to Use:  
​Connect the hardware according to the diagrams in the schematics/ folder(if you are going to use arduino while connected to your laptop or pc you dont need to connect the vin pin on arduino to your supply because your laptop will provide the electricity for arduino to work)
​Crucial: Remove the jumper on the L298N ENA pin and connect it to Arduino Pin 6 for PWM control.
​Upload the code in clean-code/ or working-code/ to your Arduino (Dont forget to change pulse_per_rotation , radius_of_disk and setpoint to suit your own disk and desired speed).
​Open Serial Monitor (9600 baud) to observe real-time data.
​
Future Improvements:  
​Migrating the project to ESP32 for dual-core processing and FreeRTOS implementation.
​Adding a Wi-Fi-based dashboard for remote setpoint adjustment.
​Implementing a high-resolution (OLED) display for on-device telemetry.
​Author: Onur Yılmaz
Year: 2026
Field: Mechatronics Engineering / Embedded Software
