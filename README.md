# WindowOpener
An ESP8266 / ESP8285 / Arduino project for automatically opening and closing a bedroom window.

I can't say we live on a busy street, but cars do pass by even at night. And because the street is not very wide, sound is reflected by the houses on the opposite side of the street. That's why we usually sleep with our bedroom window closed, albeit contrecoeur. But, we could sleep with our window open if it were to open automatically after, say, 0:30 in the morning (12:30am) because at that time no cars usually pass anymore. And if they were, we are asleep anyway so we may not hear them if they are not too loud. On this web page you can read all about how I created the hard- and software for an automated window opener. You can find all the information you need to build one yourself. It's not easy to do, but if you are somewhat experienced with tools such as a soldering iron, a saw and screwdrivers, you should be able to do it. The hardware is cheap and, as it turns out, pretty reliable (in a trial of one).

# The software

I started out with putting my requirements on paper. In a nutshell:
* I want the window to be closed when we go to bed and are trying to fall asleep. Before 0:30 cars still drive in our street, so it should stay closed before that time.
* I want to keep the window open as long as possible for fresh air, but close it before cars start driving in the morning. In practice this means that the window has to close around 6:00.
* The window must be opened by powering the motor for a programmable period. Setting the period allows us to control the position of the window.
* The window can be closed by powering the motor for a fixed period. Because the motor has end sensors, this time can be the time the motor takes to close from the fully open position.
* The system must be robust for situations where the window cannot open or close, for example when the latch has been engaged.
* It must be possible to open or close the system 'manually', e.g. by using a remote control.

# The system

On the internet I found a linear motor and I figured that would be a good starting point. I started making some sketches in PowerPoint to figure out what would be a workable angle between the window and the linear motor.

![Experimenting](https://github.com/Tsjakka/WindowOpener/blob/main/Photos/LinearActuator.gif)

# The software

For my first experiments, I stuck all the parts I thought I would need on a breadboard and started making some software. This worked very well for creating and testing the software. Before I started writing code, I made a state machine diagram that captures the behavior of the software. The image below shows this diagram. A state machine diagram shows the various states the system can be in and the conditions that trigger a transition from one state to another. Having this diagram made it a lot easier for me to write the core functionality of the software. 

![The state machine](https://github.com/Tsjakka/WindowOpener/blob/master/Photos/StateTransitionDiagram.gif)

Another important element is the microcontroller that is used to control the system. I selected an ESP8285 for this task and it works well. It is also extremely cheap, just a few euros on the well known Chinese websites. A serious downside of these boards is that there are quite a few limitations to the use of the GPIO connections. You have to be very careful when selection ports for connecting stuff. Fortunately, there are some good sites that collect information on these things, such as https://randomnerdtutorials.com/esp8266-pinout-reference-gpios/. This page describes the ESP 8266, but the info should be the same for the 8285.

I used the Arduino development environment to write the software. It's not the best IDE, but it's free and does the job.

After I created an initial version of the software, I started testing it on the test setup I made on the breadboard. This way I learned how the system behaved and where I could improve it. In this period I made a lot of changes to the software and it got better and better. 

What I also included in the software was:
* A Wi-Fi client that connects the system to the internet.
* An NTP client that reads the current time from the internet.
* A library for reading the output of the INA226 power sensor.
* A web server serving up a page that allows me open or close the window.

# The hardware

With the knowledge I gained from experimenting with the breadboard I started sketching an electronic schema for the final system. Like mentioned before, you have to be careful which GPIO to use, so along the way I found out that some things didn't work and I had to move them to another IO. In the end I got it working and this is the final schema:

![The schema](https://github.com/Tsjakka/WindowOpener/blob/main/Photos/Diagram.jpg)

The full list of hardware is:
* Universal Single Side PCB Glass Fiber 9x15cm
* ESP8285 Development Board (I used ESP-Mx DevKit)
* 12-pin header for ESP8285 (2x)
* Relay Takamisawa NA12W-K (2x)
* 1000N 150mm linear motor, DC 12V
* INA226 I2C DC Current and Voltage meter 20A Module
* LM317 DC-DC step-down DC converter circuit board (for 12V -> 5V)
* 433Mhz RF Remote with receiver
* Power supply (12V, 2A)
* Various screw headers
* Various pieces of wire

# What it looks like

Below you can see what the final window opener looks like.

![The window opener IRL](https://github.com/Tsjakka/WindowOpener/blob/main/Photos/WindowOpener.jpg)

In the Photos directory you can find some more photos and a movie showing the window in motion.
