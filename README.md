# C1 Robot Dog(my first engineering project)
## The Goal for the Project.

The Goals and specifications before going into this project were these:
The design of the robot form factor is quadruplet bot on wheel moving with four DC motor on its wheels for movement.  Moveable joints attached to a servo at both the hind legs and Rear legs making it more animated in its movement and an Ultrasonic sensor  that's used to collect distance data.

Sketch of the C1 robot and its wireless controller(2019).

![](/assets/C1_sketch_design.png) ![](assets/C1_pad_design_sketch.png)

First Attempt at building C1 (2019).

![](/assets/C1_first_attempt_2019.png)

Simulation of the control circuit (2023).

![](/assets/simuLide_control_circuit_sim_2023.png)
C1 robot and its controller (2024).

![](/assets/C1_top_side_view_2024.png)

![](/assets/C1_and_Control_pad_2024.png)


The bot has two major modes of connection, the RF mode and Bluetooth mode (controlling the bot via your smart phone)

#### RF mode.
This mode it uses a nrf24l01 module for radio communication between the remote and the bot. in this mode you can control the motion of the robot and some of its other features.

#### Bluetooth mode.
This mode would be it's default mode. in this mode you control the bot via an app. this app allows you send and receive data from the bot, trigger some commands and track it's user or follow it's user. the Bluetooth mode also has to sub-modes

#### Bluetooth Track mode
In the track mode the bot   track it's user using the received signal strength to determine how far it is from the the person controlling and where the signal is the strongest so it can move in that direction.

#### The Follow me mode
The “Follow-me” mode:  usually this mode is often done with ultrasonic sensors alone or IR sensors but the bot uses not only those in this case to follow the person but also uses the received signal from the connected Bluetooth device to track and follow the person holding the device, like a Bluetooth beacon tracker on wheels.

## Development process of the Wireless Control Pad

![](/assets/control_padV2_frontview.png)

![](/assets/control_pad_V1_back_view.png)
The radio RC(Remote Control) Pad is made to control the main circuit in the bot. The current version it features An analog joystick, 4-direction buttons, 4 customizable function buttons, A power LED and An indicator LED and a passive, NRF240L transceiver(Radio) and an Arduino Nano as the Microcontroller.

### Challenges Faced while building.

![](/assets/control_pad_v2_opened.png)

![](/assets/control_pad_veroboard_circuit.png)

Most of the issues I faced while building this control pad and with the project at large were majorly due to my inexperience with soldering and making circuits as this was my first project doing so. Here are some of the issues I faced and how I solved them.

Connecting the Radio module successfully with the microcontroller.
The NRF24L01+ transceiver uses SPI to communicate between the transceiver and the microcontroller, this meant that the SPI connection lines(MOSI, MISO, SCK, CE and CSN) between the MCU(Microcontroller Unit) and the chip has to be solid and free from any wiring interference.   

I encountered some issues with wiring, at first I used thin wires without thick insulation for the SPI lines and the soldering connected had some bridges between some of the pins, this led to the radio module not initiating or not working well. I fixed this issue by replacing the wires to   bread board jumper wires and making sure there wasn't any bridges where I didn't intend, after this, the radio module worked fine with the microcontroller.

The other factors to consider when working with the NRF24L01 module includes, adding a  buck capacitor to the power lines(I used 100uf, 16V electrolytic capacitor) but anything within the range of 10uf - 100uf is recommended to ensure stable power to the NRF24L01.

The other issue where with the other four functional buttons, These buttons were connected to some of the digital pins and configured as a pull-up pin and this caused the microcontroller to get unreasonable hot, at first I thought it was caused by a short circuit, so I built another pad to trouble the issue from start because the circuit at this point was already covered inside the pad, after building the second version making sure there was no short in the connection, I encountered the issues again (the MCU got hot) after configuring the pins the other four functional buttons were connected to as pull-up. so I used an external pull-up resistor on the pins and configured it as an input and this solved the getting hot issue.

## C1 robot mechanical structure
![](/assets/C1_Bot_v2_coveredtop.png)


![](/assets/C1_bot_V1_mechanical_structure.png)
### Mechanical Structure goal and objectives.

![](/assets/C1_bot_V2_structure_topview.png)

![](/assets/C1_top_side_view_2024.png)
A structure that is durable, functional and strong enough to support all the components in the build bringing the design closer to the first  design sketch. The feature includes:  
- A linkage extension that connects the servo mechanism to the legs(one for the hind legs and another for the rear legs)
- Slot for motors.
- Battery compartment.
- Circuit compartment.
- Compartment for the servos.

### Challenges Faced while building the structure.
Of all aspects of this build this was the aspect I somewhat enjoyed the most after the programming, as the development was progressive, I start with a small idea about the structure and then build on it.. and if I fail the only cost is the effort to draw and cut-out the desired structure again and not as costly as the circuit design due to the fact that I'm used cartons and not 3D-printing😅. PS: This changed when the gears in my servos got stripped due to over-torque.
![](/assets/damage_servo.png)

## C1 Main Control Circuit
![](/assets/simuLide_control_circuit_sim_2023.png)

![](/assets/C1_Control_circuit_v1.png)

![](/assets/C1_control_circuit_v2.png)
Simulation of  the whole (wired) control circuit made with simulide and programmed with arduinoIDE (2023). 

The control circuit was designed to control the entire robot system through either of the wireless options(the control pad or through Bluetooth). The circuit controls:
4 servo motors.
4 DC motors.
1 Ultrasonic sensor.
It also has four slots for four optical encoders that is attached each geared motors  to measure its speed.

### How it works.
![](/assets/C1_and_Control_pad_2024.png)

The Circuit can be controlled wireless through two methods as stated previously the Radio controller and a Bluetooth device.
The Radio controller when connected sends the state all the inputs(Buttons and Joystick) to the Main circuit(receiving circuit), this is then processed and a specific action is carried out. same process as the Bluetooth app, which is yet to be implemented.
Challenges Faced while building the circuit.

 I'm  currently at the second version of this Main control circuit, which was built to improve on most of the issues with the first version.

- It added more buck capacitors for servos and IC.

- Replaced the 5V regulator with a Buck Converter, which was more efficient.

- An I2C GPIO expander for provide more GPIO pins to control the motor driver(I also experimented with a multiplexer to add more pins, but it wasn't as efficient at high speeds due to signal degradation).

- Another Motor driver was added to support controlling each of the individual motor, rather than in pairs.

- And the overall soldering was better.


### Things to improve on with this Circuit design.
- Motor Driver:  the motor drivers (L293D) needs to be  optimized to a better one, a mosfet based driver, the current driver that's used in the circuit overheats very quickly within few seconds of been active, this leads to the reduced speed in a short amount of time when it's active, This is mostly likely due to its current rating(800ma).


- Limited amount of GPIO pins for peripherals and sensors: As the project grows in complexity and more sensors are added(more  ultrasonic sensors, speed encoders and so on) the esp32 only has a limited number of GPIO pins available and an I2C GPIO can only do so much,  adding another microcontroller preferably low power to control these added peripherals and sensors would be ideal.


- Also to add to this point, the Esp32 is a great microcontroller for its price and features and is ideal for Microcontroller project that are relatively compute intensive, all these comes with the caveat that it's not a beginner friendly devkit like the Arduino Uno and you may easily fry it if you don't know what you're doing. The amount of usable GPIOs reduces as more of the hardware features of the board is used, like Bluetooth, I2C, SPI and so on, so if you have multiple peripherals you want to control while still using all these features, you might want to consider adding another low power microcontroller to your circuit setup to control them.

### What I Have Achieved with the current version and the next phase.
The current version of this build was inspired by the movie A-X-L  I saw when I was younger and the robot dog by the company  wowwee chip.  My goal when I attempted building this in 2019(the year I got the idea to build this) was just to build a simple wireless controlled toy dog, which I did asides from the wireless control part and it had no circuit, just plug in and play.
 In 2024 I picked up the sketch again with the hope of completing what I had set out to do in 2019 with some added features like:

1. A wireless controller (Radio and Bluetooth).

2. A 2D obstacle avoidance system to navigate around obstacle.

3. A ‘follow-me’ mode that uses received signal from the connected Bluetooth device to track and follow the person holding the device, like a Bluetooth beacon tracker on wheels.

 Of all the four goals I have only achieved the first and I’m currently working on the second goal.

I hope to work on the three other goals in the next version I build and also use standard design -materials like acrylic boards or standard PCB design instead of the carton  and Vero-board soldering I have used so far. All I have done so far has been driven by my passion, need to improve my engineering skills and support from relatives and mentors, but the long term goal is that all I have done so far can be made into a product that can be used to teach young people about STEM, Robotics or a product that can be used for land surveillance in the industry. here’s what I have achieved for now let’s call it a version 0.1, Thank you



