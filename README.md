# CMU RI 16-711 KDC Homework 4: Kalman Filtering 
Arpit Agarwal, Adam Harley, Dhiraj Gandhi, Peiyun Hu

### Overview
We follow the formulation of [Extended Kalman Filter on Wikipedia](https://en.wikipedia.org/wiki/Extended_Kalman_filter). Please run [`part1.m`](part1.m), [`part2.m`](part2.m), and [`part3.m`](part3.m) to reproduce our results. If you run into any issues, please contact [Peiyun Hu](peiyunh@cs.cmu.edu). 

### Part 1: Handling translation and rotation

Given the measurements of 10 noisy markers at each time stamp and the markers' location in the body coordinate system, we want to estimate the state of the Center of Mass, including 3D location, velocity, orientation, and angular velocity at each time stamp. To make our estimation of state less noisy, we build an Extended Kalman Filter to explicitly factor out process noise and measurement noise. Below, we will introduce how we implement the Extended Kalman Filter, including the process, measurement, and initial states. In the end, we will show how the filter works with an example. 

**(initial) state** Each state is a 12-dimension vector, including 3 for spatial locations in 3D space, 3 for spatial velocities, 3 for orientations, and 3 for angular velocities. Each observation is a 24-dimension vector, including spatial locations of 8 markers in 3D space. Extended Kalman Filter is able to handle non-linear process and measurement, however, requires good initialization to make sure linearization is locally reasonable. We initialize the initial location of COM as the centroid location estimated from the noisy markers at the first timestamp. Similarly, we initialize the spatial velocity as the spatial velocity of the centroid, from the first timestamp to the last timestamp. We initialize orientation to be aligned with world axes and angular velocities to be all zeros. Initialization is implemented in [`hack_init_estimates.m`](hack_init_estimates.m). Since the initial location of COM is derived from the noisy marker data at only one timestamp, while the initial spatial velocity of COM is derived from all marker data, we believe the estimate of spatial velocity is more accurate and less noisy than the estimate of spatial location. Thus we assign smaller variance to the velocity estimation. 

**process** The process maps from one state to another, following the rigid body motion dynamics. The dynamics is implemented in [`do_dynamics.m`](do_dynamics.m). To estimate the covariance of state, we linearizes the dynamics and uses its Jacobian matrix. Since solving symbolic derivatives for the dynamics lead to complicated and messy equations, we decide to evaluate the derivatives numerically, which is implemented in [`relinearize.m`](relinearize.m). 

**measurement** The measurement maps from one state to its predicted observation. In this work, given a predefined/estimated configuration of markers, we can map the state of COM to the locations of markers. Such measurement is implemented in [`predict_markers.m`](predict_markers.m). To estimate the residual covariance and eventually compute the Kalman Gain, we linearizes the measurement and uses its Jacobian matrix. Unlike the motion dynamics, the measurement Jacobian is much less complicated. Thus, we compute the Jacobian using symbolic derivatives, which is scripted in [`compute_jacobian.m`](compute_jacobian.m). 

**How it works** Once we have the location of COM, we can estimate the location of markers. In the video below, we visualize how Kalman Filter is able to obtain stable estimate of markers comparing to the raw noisy data. 
[![Kalman Filter](https://img.youtube.com/vi/PUa98uWgXPY/0.jpg)](https://www.youtube.com/watch?v=PUa98uWgXPY "Kalman Filter")

What happens behind the scene is that we gradually correct the state prediction based on the observations with a proportion of the Kalman Gain. Ideally, we are less likely to make big changes as we gather more evidence. We visualize the Kalman Gain below, which drops down close to zero after a couple of timestamps. 
![Kalman Gains](/part1/p1n00_kalman_gain.png)

Please find our results of part 1 under [`part1`](/part1), which contains state estimates (p1a0x) and visualizations. For an example, we show visualization of p1n00 below. 
![Spatial Track](/part1/p1n00_spatial_track.png)
![Spatial Variance](/part1/p1n00_spatial_var.png)
![Angular Track](/part1/p1n00_angular_track.png)
![Angular Variance](/part1/p1n00_angular_var.png)


### Part 2: Handling occlusion 
To handle occlusion, we need to make two modifications to our solution of Part 1. 
- Make sure we don't consider occluded markers when computing the initial state, including initial location and velocity; [(`hack_init_estimates.m`)](hack_init_estimates.m)
- Make sure the rows of measurement Jacobian that correspond to occluded markers are zeros. [(`predict_markers.m`)](predict_markers.m) 

Please find our results of part 2 under [`part2`](/part2), which contains state estimates (p2a0x) and visualizations. For an example, we show visualization of p2n00 below. 
![Spatial Track](/part2/p2n00_spatial_track.png)
![Spatial Variance](/part2/p2n00_spatial_var.png)
![Angular Track](/part2/p2n00_angular_track.png)
![Angular Variance](/part2/p2n00_angular_var.png)

### Part 3: Estimating marker locations
To estimate marker locations along with the state of COM, we simply add marker locations (in body coordinate system) into our state variable. There are two modifications we need to make based on the solution of Part 1: 
- Make sure we initialize the additional dimensions of the state and also specify variance of esitmates. [(`doit_slam.m`)](doit_slam.m)
- Instead of using predefined marker body configuration, we always refer to the state variable for markers' relative location. Thus, when computing the Jacobian in measurement, we need to compute the derivatives w.r.t. the marker location as well. [(`predict_markers_slam.m`)](predict_markers_slam.m)

Please find our results of part 3 under [`part3`](/part3), which contains state estimates (p3a0x) and visualizations. For an example, we show visualization of p3n00 below. 
![Spatial Track](/part3/p3n00_spatial_track.png)
![Spatial Variance](/part3/p3n00_spatial_var.png)
![Angular Track](/part3/p3n00_angular_track.png)
![Angular Variance](/part3/p3n00_angular_var.png)
![Marker Locations](/part3/p3n00_marker_body.png)
