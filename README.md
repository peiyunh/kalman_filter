# CMU RI 16-711 KDC 
## Homework 4: Kalman Filtering 

### Part 1: Handling translation and rotation
Given the measurements of 10 noisy markers at each time stamp and the markers' location in the body coordinate system, we want to estimate the state of the Center of Mass, including 3D location, velocity, orientation, and angular velocity at each time stamp. To make our estimation of state less noisy, we build a Kalman Filter to explicitly factor out process noise and measurement noise. Below, we will introduce how we created the Kalman Filter, including the process, measurement, and initial states. In the end, we will show how the filter works with an example. 

The Kalman Filter includes two important components: process and measurement. In this work, the process models the motion dynamics of the rigid body and the measurement models the mapping from the state of COM to observations. We formulate these two components as: 
\begin{equation}
a = a + 1
\end{equation}

### Part 2: Handling occlusion 

### Part 3: Estimating marker locations
