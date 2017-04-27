# CMU RI 16-711 KDC 
## Homework 4: Kalman Filtering 

### Part 1: Handling translation and rotation
Given the measurements of 10 noisy markers at each time stamp and the markers' location in the body coordinate system, we want to estimate the state of the Center of Mass, including 3D location, velocity, orientation, and angular velocity at each time stamp. To make our estimation of state less noisy, we build an Extended Kalman Filter to explicitly factor out process noise and measurement noise. Below, we will introduce how we implement the Extended Kalman Filter, including the process, measurement, and initial states. In the end, we will show how the filter works with an example. 

We follow the formulation of [Extended Kalman Filter on Wikipedia](https://en.wikipedia.org/wiki/Extended_Kalman_filter). 

**(initial) state** Each state is a 12-dimension vector, including 3 for spatial locations in 3D space, 3 for spatial velocities, 3 for orientations, and 3 for angular velocities. Each observation is a 24-dimension vector, including spatial locations of 8 markers in 3D space. Extended Kalman Filter is able to handle non-linear process and measurement, however, requires good initialization to make sure linearization is locally reasonable. We initialize the initial location of COM as the centroid location estimated from the noisy markers at the first timestamp. Similarly, we initialize the spatial velocity as the spatial velocity of the centroid, from the first timestamp to the last timestamp. We initialize orientation to be aligned with world axes and angular velocities to be all zeros. Since the initial location of COM is derived from the noisy marker data at only one timestamp, while the initial spatial velocity of COM is derived from all marker data, we believe the estimate of spatial velocity is more accurate and less noisy than the estimate of spatial location. Thus we assign smaller variance to the velocity estimation. 

**process** The process maps from one state to another, following the rigid body motion dynamics. The dynamics is implemented in [`do_dynamics.m`](https://github.com/peiyunh/kalman_filter/blob/master/do_dynamics.m). To estimate the covariance of state, we linearizes the dynamics and uses its Jacobian matrix. Since solving symbolic derivatives for the dynamics lead to complicated and messy equations, we decide to evaluate the derivatives numerically, which is implemented in [`relinearize.m`](https://github.com/peiyunh/kalman_filter/blob/master/relinearize.m). 

**measurement** The measurement maps from one state to its predicted observation. In this work, given a predefined/estimated configuration of markers, we can map the state of COM to the locations of markers. Such measurement is implemented in [predict_markers.m](https://github.com/peiyunh/kalman_filter/blob/master/predict_markers.m). To estimate the residual covariance, we linearizes the measurement and uses its Jacobian matrix. Unlike the motion dynamics, the measurement Jacobian is much less complicated. Thus, we compute the Jacobian using symbolic derivatives, which is scripted in [compute_jacobian.m](https://github.com/peiyunh/kalman_filter/blob/master/compute_jacobian.m). 

**How it works** We show 

### Part 2: Handling occlusion 

### Part 3: Estimating marker locations
