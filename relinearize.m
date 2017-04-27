function F = relinearize( state_estimate, I_world );

global n_x I_body

d = 1e-3;
F = zeros(n_x, n_x); 
for i = 1:n_x
	state_lo = state_estimate; 
	state_lo(i) = state_lo(i) - d;
   rot_mat_lo = compute_rot_mat(state_lo); 
   I_world_lo = rot_mat_lo * I_body * rot_mat_lo'; 
	state_lo_new = do_dynamics(state_lo, I_world_lo); 

	state_hi = state_estimate; 
	state_hi(i) = state_hi(i) + d; 
   rot_mat_hi = compute_rot_mat(state_hi); 
   I_world_hi = rot_mat_hi * I_body * rot_mat_hi';  
	state_hi_new = do_dynamics(state_hi, I_world_hi); 
	
	F(:,i) = (state_hi_new - state_lo_new) / (2*d);
end	
