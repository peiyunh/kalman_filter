function doit(data_file)
close all;

if nargin < 1
  error('please specify the data file');
end
output_file = strrep(data_file, 'n', 'a');

addpath tools;

% load marker data file
markersn = load(data_file);

global T
% time step (length of time between measurements)
T = 0.1;

% number of filter states
global n_x
n_x = 12;
% number of markers
global n_m
n_m = 8;
% number of samples
n_t = length(markersn);

% marker locations in body coordinates
global markers_body
markers_body = [
  -1.5   -3.0   -4.5
  -1.5   -3.0    1.5
  -1.5    1.0   -4.5
  -1.5    1.0    1.5
  0.5   -3.0   -4.5
  0.5   -3.0    1.5
  0.5    1.0   -4.5
  0.5    1.0    1.5
  ];

global I_body 
I_body = diag([1.571428571428571, 5.362637362637362, 7.065934065934067]);

% set up data arrays
quaternions_before_measurement( n_t, 4 ) = 0;
states_before_measurement( n_t, n_x ) = 0;
quaternions_before_process_update( n_t, 4 ) = 0;
states_before_process_update( n_t, n_x ) = 0;
marker_errors( n_t, n_m*3 ) = 0;
kalman_gains( n_t, 3 ) = 0;
variance_estimates_before_measurement( n_t, n_x ) = 0;
variance_estimates_before_process_update( n_t, n_x ) = 0;

% Initial estimates. Since EKF needs to get nonlinear part to be close
[ quaternion, state_estimate ] = hack_init_estimates( markersn );

% Initial condition noise model
variance_estimate = 5e-2*eye( n_x );
% we are more confident in our velocity initialization (with centroid)
variance_estimate(1,1) = 1;
variance_estimate(2,2) = 1;
variance_estimate(3,3) = 1;
variance_estimate(4,4) = 1e-2;
variance_estimate(5,5) = 1e-2;
variance_estimate(6,6) = 1e-2;
variance_estimate(7,7) = 1;
variance_estimate(8,8) = 1;
variance_estimate(9,9) = 1;
variance_estimate(10,10) = 1;
variance_estimate(11,11) = 1;
variance_estimate(12,12) = 1;

Q = diag( ones( n_x, 1 ) );
Q(1,1) = 1e-6;
Q(2,2) = 1e-6;
Q(3,3) = 1e-6;
Q(4,4) = 1e-10;
Q(5,5) = 1e-10;
Q(6,6) = 1e-10;
Q(7,7) = 1e-3;
Q(8,8) = 1e-3;
Q(9,9) = 1e-3;
Q(10,10) = 1e-3;
Q(11,11) = 1e-3;
Q(12,12) = 1e-3;

% Measurement noise model: marker variance
R = diag( ones( n_m*3, 1 ) );

for index = 1:length(markersn)
  % Compute rotation matrix and update quaternion based on current state
  [ quaternion, rot_mat ] = update_rotation( state_estimate, quaternion );
  
  % Compute measurement error
  [ marker_error, H ] = predict_markers( quaternion, state_estimate, index, markersn );
  
  % Store values
  quaternions_before_measurement( index, : ) = quaternion;
  states_before_measurement( index, : ) = state_estimate;
  marker_errors( index, : ) = marker_error;
  for i = 1:n_x
    variance_estimates_before_measurement( index, i ) = variance_estimate( i, i );
  end
  
  % Measurement update
  % should not use inv(), solve set of equations instead.
  kalman_gain = variance_estimate*H'*inv(H*variance_estimate*H' + R);
  for i = 1:3
    kalman_gains( index, i ) = kalman_gain( i, i );
  end
  state_estimate = state_estimate - kalman_gain*marker_error;
  variance_estimate = (eye(n_x) - kalman_gain*H)*variance_estimate;
  
  % Compute rotation matrix and update quaternion based on current state
  [quaternion, rot_mat] = update_rotation( state_estimate, quaternion );

  %% 
  I_world = rot_mat * I_body * rot_mat';  
  
  % Relinearize
  F = relinearize( state_estimate, I_world );
  
  % Store values
  quaternions_before_process_update( index, : ) = quaternion;
  states_before_process_update( index, : ) = state_estimate;
  for i = 1:n_x
    variance_estimates_before_process_update( index, i ) = variance_estimate( i, i );
  end
  
  % Process update.
  state_estimate = do_dynamics( state_estimate, I_world );
  variance_estimate = F*variance_estimate*F' + Q;
end

plotit(states_before_measurement, states_before_process_update, ...
  quaternions_before_measurement, quaternions_before_process_update, ...
  variance_estimates_before_measurement, variance_estimates_before_process_update, ...
  kalman_gains, data_file);

output = [states_before_measurement(:,1:6) quaternions_before_measurement(:,1:4) states_before_measurement(:,10:12)];
dlmwrite(output_file, output);