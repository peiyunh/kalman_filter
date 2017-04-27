function state_estimate = do_dynamics( state_estimate, I_world );

% global T

% w = state_estimate(10:12); 
% wdot = -inv(I_world) * cross(w, I_world*w);

% A = [ 1 0 0 T 0 0 0 0 0 0 0 0 
% 	    0 1 0 0 T 0 0 0 0 0 0 0 
% 	    0 0 1 0 0 T 0 0 0 0 0 0 
% 	    0 0 0 1 0 0 0 0 0 0 0 0 
% 	    0 0 0 0 1 0 0 0 0 0 0 0 
% 	    0 0 0 0 0 1 0 0 0 0 0 0
% 	    0 0 0 0 0 0 1 0 0 T 0 0 
% 	    0 0 0 0 0 0 0 1 0 0 T 0 
% 	    0 0 0 0 0 0 0 0 1 0 0 T 
% 	    0 0 0 0 0 0 0 0 0 1 0 0 
% 	    0 0 0 0 0 0 0 0 0 0 1 0
% 	    0 0 0 0 0 0 0 0 0 0 0 1 ]; 
% B = [ 0 
% 	  0 
% 	  0
% 	  0
% 	  0 
% 	  0
% 	  0
% 	  0
% 	  0
% 	  T*wdot(1)
% 	  T*wdot(2)
% 	  T*wdot(3)] ; 
% state_estimate = A * state_estimate + B; 
global T

wdot=-inv(I_world)*cross(state_estimate(10:12),I_world*state_estimate(10:12));

% update spatial velocity first 
state_estimate(4) = state_estimate(4);
state_estimate(5) = state_estimate(5);
state_estimate(6) = state_estimate(6);
state_estimate(1) = state_estimate(1) + T*state_estimate(4);
state_estimate(2) = state_estimate(2) + T*state_estimate(5);
state_estimate(3) = state_estimate(3) + T*state_estimate(6);
% update angular velocity first
state_estimate(10) = state_estimate(10)+T*wdot(1);
state_estimate(11) = state_estimate(11)+T*wdot(2);
state_estimate(12) = state_estimate(12)+T*wdot(3);
state_estimate(7) = state_estimate(7)+T*state_estimate(10);
state_estimate(8) = state_estimate(8)+T*state_estimate(11);
state_estimate(9) = state_estimate(9)+T*state_estimate(12);
% state_estimate(13:36) = state_estimate(13:36);

end % :(
