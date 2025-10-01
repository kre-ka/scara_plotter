# SCARA Plotter

This piece of software makes (or will make, as it's a WIP) [SCARA](https://en.wikipedia.org/wiki/SCARA) robotic arm draw shapes defined by splines. Made in C for STM32 microcontroller (not yet integrated).

## Why SCARA

Cartesian would be better, but SCARA is cooler, and it's most of all a fun project.

## Inner machinations

See [main.c](src/main.c).

- Create and initialize a `Manipulator` struct with SCARA manipulator parameters: member lengths and joint angle limits
- display robot work area
- create a parametric `CubicCurve` path to follow
- compute a time-spatial `Trajectory` from given `CubicCurve`, speed (initial, target and final) and acceleration (here is a cool arc lenght parametrization problem solution, so if you are to check any of this code out, check  `make_p_t_map_table` at [curve.c](src/curve.c))
- execute the `Trajectory` (one point at the time) by interpolating and calculating inverse kinematics for each point (and validating whether each point is within manupulator range)

## TODO
- splines handling with trajectory computation spread over time
- STM32 integration with motor control
- G-code parsing
- other path primitives support (straight lines, circles)
- vertical axis control
- there's certainly more