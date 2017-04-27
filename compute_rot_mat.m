function R = compute_rot_mat(state_estimate)
	
ax = state_estimate(7); sx = sin(ax); cx = cos(ax); 
ay = state_estimate(8); sy = sin(ay); cy = cos(ay); 
az = state_estimate(9); sz = sin(az); cz = cos(az); 

R = [cy*cz           -cy*sz          sy
	 cx*sz+cz*sx*sy  cx*cz-sx*sy*sz  -cy*sx
	 sx*sz-cx*cz*sy  cx*sy*sz+cz*sx  cx*cy ]; 