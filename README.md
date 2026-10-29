
# FLL Pybricks Robot Driver

This is the custom robot navigation code that me and my team used for our FIRST LEGO League (FLL) robot. We spent countless hours in the robotics lab at VŠB Ostrava tuning this thing, and it ended up carrying us through the regionals and all the way to the Czech National Championship in Prague. 

We were tied 470 to 470 in the finals and ended up losing the title 480 to 485 by just 5 points because of a risky 5-second buzzer-beater mission call, but this code was the absolute backbone of our run and honestly one of the things I am most proud of from that time.

## What is this?
Basically instead of using the standard block coding or relying only on default movement, I wrote a custom Python driver running on Pybricks for the LEGO Spike Prime Hub. 

The main goals were:
* **Custom Acceleration Ramp**: Acceleration and deceleration profiles so the wheels don't slip and lose track on the mat.
* **Gyro Straight Driving**: A P-loop checking the internal IMU heading and actively correcting the motors so the robot drives in a dead straight line.
* **Gyro Turns**: Smooth turns that ramp up and down based on the gyro, with a calibration loop at the end to settle small angle errors.
* **Garbage Collection tweaks**: Disabling MicroPython's garbage collector during runs (`gc.disable()`) and manually calling it between moves so the robot doesn't randomly stutter mid-drive.
* **LED Status Lights**: Hub lights change colors (Violet = accelerating, Blue = cruising, Red = braking, Cyan/Green = settling) so you can see what the robot is doing in real time.

## Hardware setup:
* **Hub**: LEGO SPIKE Prime / Robot Inventor Hub (running Pybricks firmware)
* **Left Drive Motor**: Port B (Counter-clockwise)
* **Right Drive Motor**: Port F (Clockwise)
* **Attachment Motors**: Port A & Port E (Modules/tools)
* **Wheel diameter**: 56 mm
* **Track width**: 160 mm

## How to use:
Just flash Pybricks onto your Prime Hub and run `main.py` using the [Pybricks web IDE](https://code.pybricks.com/).

Basic navigation commands:
```python
# Move 10 cm forward at 800 speed
move_cm(10, 800)

# Turn 90 degrees counter-clockwise
turn_gyro(-90, 500)

# Relative turn
turn_gyro(20, 200)

# Control left/right attachment modules
moduleL(90, 300)
```

## Things to keep in mind:
* This was custom tuned to our specific robot's center of mass, wheel size, and gear ratios, so if you're using this on a different build you will probbably need to tweak the `kp` values and ramp distances.
* Don't expect some super clean corporate codebase, this was written with blood, sweat, and tears during late lab nights, but it works and it's fast.
