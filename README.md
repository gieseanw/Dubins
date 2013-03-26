Dubins
======
This project is for computing the Dubin's shortest paths for arbitrary turning radius.

The Dubin's car is a simple robot that can only move forward at unit speed and steering angle.

In this iteration, the steering angle and speed can be variable.

I use Eulerian integration to integrate the solved trajectories. Eulerian integration simply steps things forward by a small
timestep delta. It is not a stable integration technique, but it's simple and fast. Those wanting a better technique can
look into Runge Kutta. Generally, if the final state is not near enough to where you wanted the car to go, decrease
DELTA to 0.01 or less. 

Velocity is faked in a manner -- I simply integrate as many steps as the velocity specified. It will also vary based on
DELTA's value. So if you modify DELTA, you'll probably have to modify your velocity.

The shortest path solver is defined in Dubins.h and Dubins.cpp. There's only a single public function to call, and you
must pass it the wheelbase of your car as well as the car's turning radius. These together will define the car's maximum steering
angle. You could just modify the function to only take a steering angle, but if you're trying to model a car whose specs
you got off the internet, generally you're only provided with those two values, so you need to solve for the steering
angle anyway.

All the utility functions are defined in Includes.h and Includes.cpp

When an AgentController (responsible for velocity, wheelbase, and solving trajectories for a specific agent) is given a
query to solve, it calls the Dubins shortest path solver to get a trajectory. A DubinsTrajectory consists of a vector of
controls. A control is simply a steering angle that one should apply for a number of timesteps. The timesteps are
computed using DELTA.

