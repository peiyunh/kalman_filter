function state_estimate = do_dynamics( state_estimate, I_world );

global T

I_world
pause;

w = state_estimate(10:12); 
wdot = -inv(I_world) * cross(w, I_world*w);

A = [ 1 0 0 T 0 0 0 0 0 0 0 0 
	    0 1 0 0 T 0 0 0 0 0 0 0 
	    0 0 1 0 0 T 0 0 0 0 0 0 
	    0 0 0 1 0 0 0 0 0 0 0 0 
	    0 0 0 0 1 0 0 0 0 0 0 0 
	    0 0 0 0 0 1 0 0 0 0 0 0
	    0 0 0 0 0 0 1 0 0 T 0 0 
	    0 0 0 0 0 0 0 1 0 0 T 0 
	    0 0 0 0 0 0 0 0 1 0 0 T 
	    0 0 0 0 0 0 0 0 0 1 0 0 
	    0 0 0 0 0 0 0 0 0 0 1 0
	    0 0 0 0 0 0 0 0 0 0 0 1 ]; 
B = [ 0 
	  0 
	  0
	  0
	  0 
	  0
	  0
	  0
	  0
	  T*wdot(1)
	  T*wdot(2)
	  T*wdot(3)] ; 

state_estimate = A * state_estimate + B; 