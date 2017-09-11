# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## The Model

We use Model Predictive Control to minimize the cross track and orientation errors of the vehicle with respect to a reference path drawn by a polynomial.  The state or the desired positioning of the vehicle is set to the center of the road.  

We predict the state and apply the control inputs, once the vehicle moves into the predicted state, we repeat the process of predicting the state and applying the control inputs (this creates our feedback loop).  

**State**

- x, y = cars position in coordinates
- psi = vehicles orientation
- v = velocity

**Actuators**

To mimic realistic driving behavior,
- Lower and Upperbounds for steering angles are constrained within -25 and 25 degrees converted to radians (MPC.cpp, line 180-186).
- Acceleration can be between -1 and 1, <0 implying braking and >0 implying acceleration (MPC.cpp, line 188-193).

**Cost Function**

    for (int t = 0; t < N; t++) {
      fg[0] += CppAD::pow(vars[cte_start + t] , 2);
      fg[0] += CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += CppAD::pow(vars[v_start + t], 2);
    }

We want cte and epsi to be close to 0 as possible.  Using Ipopt the function is optimized through calculating the aggrecate cost.






## Timestep Length and Elapsed Duration (N & dt)

**N (timestep length)**

High values of N makes the cost function much more computational and more difficult to solve for the optimal solution.  I found that a timestep of 10 is a good medium to gather the necessary information to plot a good path.  Given the environment, it wouldn't be efficient to have higher values of N to predict the state further into the future.  

**dt (elapsed duration between timesteps)**

Ideally, dt should be minimized as much as possible.  Without latency, the car runs smoothly at a dt value of 0.05.  However, adding latency caused the car to oscillate left and right excessively because it wasn't able to quickly process the actuations optimally.  Although the car is still able to make it through the track, it would be a very unpleasant experience any passenger.  To compensate for the additional latency, adding 100ms to dt (0.15 dt) provides sufficient time elapses between actuations.    

## Polynomial Fitting and MPC Preprocessing

- The MPC trajectory is displayed in green
- A polynomial fitted reference path is displayed in yellow

## Model Predictive Control with Latency

Latency is handled by predicting the state of x, y, psi and velocity 100ms in the future, and using this information to process through the MPC solver.  This way the control inputs are synced 100ms in the future.  
