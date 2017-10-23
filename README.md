# PID Controller Project
Self-Driving Car Engineer Nanodegree Program

Sylvana Alpert

---

In this project a PID controller was implemented to steer the vehicle and keep it in the center of the road at all times. Although not required, another PID controller was used to control the speed of the car, with a target velocity of 60 mph.

The PID algorithm requires tuning of its parameters (Kp, Ki, Kd) in order to function correctly. This was initially done through manual tuning and later fine tuned using the twiddle algorithm. Applying twiddle without prior manual tuning was not possible since the vehicle would leave the road, preventing any further optimization.

As a first step, the steering PID was tuned with constant speed, then the throttle PID was implemented and tuned while keeping the steering PID fixed. Each controller was optimized in turns, while the other one was kept constant with the optimal parameters found in the previous step. This process was repeated until satisfactory results were achieved. Tuning was performed while the SSE was evaluated over a full lap of the track, to avoid easy segments of the track to have influence on the algorithm.

The throttle PID uses the absolute value of the cross-track error (CTE) to correct the speed, as positive or negative CTEs should have the same effect: the vehicle accelerates when is correctly positioned in the center of the road and decelerates when it deviates to the sides. In addition, because of the symmetry around the center, Ki is set to zero in this controller, to prevent this term from growing indefinitely.

The effects of the different parameters on the steering PID are as follows: A larger Kp increases the responsiveness to the error but causes overshooting of the vehicle (leading to strong oscillations from side to side). Kd helps counteract those oscillations and when properly tuned, it causes the vehicle to slowly approach the center line. Ki has a subtle effect in this implementation, reducing the CTE around the curves and helping achieve a smoother driving experience.


## Basic Build Instructions

1. Make a build directory: `mkdir build && cd build`
2. Compile: `cmake .. && make`
3. Run it: `./pid`.
