# Micromouse
Micromouse was designed for my senior project at CSULA for the College of Electrical and Computer Engineering. For all intentand purposes uploaded to introduce people new to embedded software progrogramming and system design, trying to understand how to create and intorductory Micromouse.

Microcontroller used is a Teensy 3.2, hence using of some clock configuration for NVIC registers on the ARM-M3.

Search algorithm used is a modified flood fill in order to solve a 10 x 10 maze instead of a 16 x 16 cell maze for test purposes
where the bottom right of the maze is the start point and the top left is the end goal.

FloodFill Simulation uses self generated mazes from various past micromouse competitions.
Flood fill reconfiguration only occurs locally in order to save computation time.
