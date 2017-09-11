# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## The Model

## Timestep Length and Elapsed Duration (N & dt)

**N (timestep length)**

High values of N makes the cost function much more computational and more difficult to solve for the optimal solution.  I found that a timestep of 10 is a good medium to gather the necessary information to plot a good path.  Given the environment, it wouldn't be efficient to have higher values of N to predict the state further into the future.  

**dt (elapsed duration between timesteps)**

Ideally, dt should be minimized as much as possible.  Without latency, the car runs smoothly at a dt value of 0.05.  However, adding latency caused the car to oscillate left and right excessively because it wasn't able to quickly process the actuations optimally.  Although the car is still able to make it through the track, it would be a very unpleasant experience any passenger.  To compensate for the additional latency, adding 100ms to dt (0.15 dt) provides sufficient time elapses between actuations.    

## Polynomial Fitting and MPC Preprocessing

- The MPC trajectory is displayed in green
- A polynomial fitted reference path is displayed in yellow

## Model Predictive Control with Latency

Latency is handled with predicting the state of x, y, psi and velocity 100ms in the future, and then processed through the MPC solver.
