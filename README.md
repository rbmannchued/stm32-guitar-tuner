# STM32 Electric Guitar Tuner

libopencm3 and CMSIS DSP based guitar tuner.\
resolution of 0.97hz and great efficiency using stm32f411.
![Tuner Working Gif](/img/gifTunerCut.gif)

# Usage
After you clone the repo with submodules, \
Compile the code then flash the binary to your ST-link:
```
cd guitarTuner
make
make flashbin
```
After that you will probably have to unplug and plug again your ST-link,and that is it\
now if your connections are right you will see the nearest note, frequency detected and a bar that shows \
how far you are from the note.

# Connections
```
Display SDA >> B7 stm32 pin
Display SCL >> B6 stm32 pin 
Display VCC and GND >> stm32 VCC and GND
Guitar Signal Amplified >> A1 stm32 pin
Guitar Signal GND >> stm32 GND
```
It's recommended to put a 4.7k resistor between SDA and VCC and another between SCL and VCC. 
# How it works
## Signal Amplification
First, the weak signal from the magnetic guitar pickups needs to be amplified so that the microcontroller can understand it.\
Using lm358 opamp is a good idea, you can follow the diagram:

![Opamp wiring diagram](/img/opAmpDiagram.png)




