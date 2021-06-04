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

Steve Hobley 2012 - www.stephenhobley.com
