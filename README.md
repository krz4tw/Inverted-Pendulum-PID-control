# Inverted-Pendulum-PID-control
The goal is to create an PID controller for an inverted pendulum built out of a salvaged Inkjet printer.

It´s using linear and rotational quadrature encoders found in the printer to determine the linear position of the cart and
the angle of the pendulum. Its also using a salavaged photointerrupter as a limit switch.
The motor is driven by an L298N Motordriver by an external power supply.

A video of the system in action along with further explenations and questions can be found here:

























This Project is using the PID-Library by Brett Beauregard
***************************************************************
* Arduino PID Library - Version 1.1.1
* by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
*
* This Library is licensed under a GPLv3 License
***************************************************************

 - For an ultra-detailed explanation of why the code is the way it is, please visit: 
   http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

 - For function documentation see:  http://playground.arduino.cc/Code/PIDLibrary
