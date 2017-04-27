function [quaternion, R] = update_rotation(state_estimate, quaternionprev)

% ax = state_estimate(7); sx = sin(ax); cx = cos(ax); 
% ay = state_estimate(8); sy = sin(ay); cy = cos(ay); 
% az = state_estimate(9); sz = sin(az); cz = cos(az); 

% R = [cy*cz           -cy*sz          sy
% 	 cx*sz+cz*sx*sy  cx*cz-sx*sy*sz  -cy*sx
% 	 sx*sz-cx*cz*sy  cx*sy*sz+cz*sx  cx*cy ]; 
R = compute_rot_mat(state_estimate);

quaternion = dcm2quat(R); 
quaternion = reshape(quaternion, [], 1);

% if abs(quaternion(1)-quaternionprev(1))+abs(quaternion(2)-quaternionprev(2))+abs(quaternion(3)-quaternionprev(3))+abs(quaternion(4)-quaternion(4))>=.5
%     quaternion= -quaternion;
% end
