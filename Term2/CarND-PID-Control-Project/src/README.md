# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---
## Refection on the project
This project shows how a simple PID controller can be used to maintain the trajectory of a car roughly centered on a track. 
The steering value is calculated as follows:
steering = -Kp*CTError - Kd*d(CTError)/dt -Ki*sum(CTError)

The three coefficients Kp, Kd, Ki were chosen experimentally after understanding the impact of each one of them on the vehicle position on the track. 

* Kp (proportional term): increasing Kp makes the vehicle respond really quickly to any error. It tends to create make the trajectory very jittery.
* Kd (derivative term): the more Kd is increased, the more centered the vehicle was. However, when Kd is increased, the vehicle tends to have some small overshoots around the expected position (steering angle oscillating between 25deg and -25deg even when the car was well centered)
* Ki (integral term): Ki needs to be very small or else the vehicle starts oscillating without converging on the center of the track.
## Dependencies

* cmake >= 3.5
* make >= 4.1
* gcc/g++ >= 5.4
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 


## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.


