function [quaternion, R] = update_rotation(state_estimate, quaternionprev)

R = compute_rot_mat(state_estimate);

quaternion = dcm2quat(R); 
quaternion = reshape(quaternion, [], 1);