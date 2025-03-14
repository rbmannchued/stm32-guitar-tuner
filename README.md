# STM32 Electric Guitar Tuner

libopencm3 and CMSIS DSP based guitar tuner



## How it works
First, the weak signal from the magnetic guitar pickups needs to be amplified so that the microcontroller can understand it.\
Using lm358 opamp is a good idea, you can follow the diagram:

![Opamp wiring diagram](/img/opAmpDiagram.png)




