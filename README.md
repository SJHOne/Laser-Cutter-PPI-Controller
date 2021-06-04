# Laser-Cutter-PPI-Controller
A inline control signal manipulator that maintains constant pulse per inch, accounting for laser cutter head velocity.

Based on the work published by dirktheng :
http://www.buildlog.net/blog/2011/12/getting-more-power-and-cutting-accuracy-out-of-your-home-built-laser-system

Original routine and hardware replaced by external interrupts on lines D2 (Step X) and D3 (Step Y)
Direction pin ignored (distances travelled are positive absolute)
Laser input on pin 6, PPI output on pin 7
LED Output (ready) on pin 8 - code reads config values from A0 and A1 once at startup, press reset to re-read 
(disable laser power first!)

Uses the digitalPin fast read/write library
Uses MsTimer2 library

Version 0.1
Replaced external hardware with interrupt driven counters
Version 0.2
Optimized code
Version 0.3
Replaced millis() with interrupt timer
Version 0.4
Moved pin assignments
Version 0.5
Used function pointer to define routine to use

Steve Hobley 2012 - www.stephenhobley.com
