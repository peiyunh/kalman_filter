function F = relinearize( state_estimate, I_world );

global n_x I_body
d = 1e-3;
F = zeros(n_x, n_x); 

% global I_world
% update_rotation(state_estimate); 
% I_world

% F(1:9, :) = [1 0 0 T 0 0 0 0 0 0 0 0;
%              0 1 0 0 T 0 0 0 0 0 0 0;
%              0 0 1 0 0 T 0 0 0 0 0 0;
%              0 0 0 1 0 0 0 0 0 0 0 0;
%              0 0 0 0 1 0 0 0 0 0 0 0;
%              0 0 0 0 0 1 0 0 0 0 0 0;
%              0 0 0 0 0 0 T 0 0 1 0 0;
%              0 0 0 0 0 0 0 T 0 0 1 0;
%              0 0 0 0 0 0 0 0 T 0 0 1;
%             ];
% for i = 10:12
%    state_lo = state_estimate; 
%    state_lo(i) = state_lo(i) - d; 
%    update_rotation(state_lo);  % set I_world based on new state
%    state_lo_new = do_dynamics(state_lo); 

%    state_hi = state_estimate; 
%    state_hi(i) = state_hi(i) + d; 
%    update_rotation(state_hi);  % set I_world based on new state
%    state_hi_new = do_dynamics(state_hi); 
   
%    F(:,i) = (state_hi_new - state_lo_new) / (2*d);
% end   

% update_rotation(state_estimate); 

% for i = 1:n_x 
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

% % reset I_world
% update_rotation(state_estimate);
% % I_world
% F 
% pause;

% % keyboard;

% return; 

% global T 

% Ixx=I_world(1,1);
% Ixy=I_world(1,2);
% Ixz=I_world(1,3);
% Iyx=I_world(2,1);
% Iyy=I_world(2,2);
% Iyz=I_world(2,3);
% Izx=I_world(3,1);
% Izy=I_world(3,2);
% Izz=I_world(3,3);

% % wx=state_estimate(7);
% % wy=state_estimate(8);
% % wz=state_estimate(9);
% wx = state_estimate(10); 
% wy = state_estimate(11); 
% wz = state_estimate(12);

% % a=(Ixx*Iyy-Iyz^2)*(-Ixz*wy+Ixy*wz)+(Ixz*Iyy-Ixy*Iyz)*(2*Ixy*wx-Ixx*wy+Iyy*wy+Iyz*wz)+(Ixx*Ixy-Ixz*Iyz)*(-2*Ixz*wx-Iyz*wy+(Ixx-Izz)*wz);
% % b=(Ixx*Ixy-Ixz*Iyz)*(-Iyz*wx+Ixy*wz)+(Ixz*Iyy-Ixy*Iyz)*(-Ixx*wx+Iyy*wx-2*Ixy*wy+Ixz*wz)+(Ixx*Iyy-Iyz^2)*(-Ixz*wx-2*Iyz*wy+(Iyy-Izz)*wz);
% % c=(Ixz*Iyy-Ixy*Iyz)*(Iyz*wx+Ixz*wy)+(Ixx*Ixy-Ixz*Iyz)*(Ixx*wx-Izz*wx+Ixy*wy+2*Ixz*wz)+(Ixx*Iyy-Iyz^2)*(Ixy*wx+Iyy*wy-Izz*wy+2*Iyz*wz);
% % d=-(Ixx*Ixy-Ixz*Iyz)*(-Ixz*wy+Ixy*wz)+(-Ixy*Ixz+Ixx*Iyz)*(2*Ixy*wx-Ixx*wy+Iyy*wy+Iyz*wz)-(Ixx^2-Ixz^2)*(-2*Ixz*wx-Iyz*wy+(Ixx-Izz)*wz);
% % e=-(Ixx^2-Ixz^2)*(-Iyz*wx+Ixy*wz)+(-Ixy*Ixz+Ixx*Iyz)*(-Ixx*wx+Iyy*wx-2*Ixy*wy+Ixz*wz)-(Ixx*Ixy-Ixz*Iyz)*(-Ixz*wx-2*Iyz*wy+(Iyy-Izz)*wz);
% % f=(-Ixy*Ixz+Ixx*Iyz)*(Iyz*wx+Ixz*wy)-(Ixx^2-Ixz^2)*(Ixx*wx-Izz*wx+Ixy*wy+2*Ixz*wz)-(Ixx*Ixy-Ixz*Iyz)*(Ixy*wx+Iyy*wy-Izz*wy+2*Iyz*wz);
% % g=(Ixz*Iyy-Ixy*Iyz)*(Ixz*wy-Ixy*wz)+(Ixy^2-Ixx*Iyy)*(2*Ixy*wx-Ixx*wy+Iyy*wy+Iyz*wz)-(Ixy*Ixz-Ixx*Iyz)*(-2*Ixz*wx-Iyz*wy+(Ixx-Izz)*wz);
% % h=-(Ixy*Ixz-Ixx*Iyz)*(-Iyz*wx+Ixy*wz)+(Ixy^2-Ixx*Iyy)*(-Ixx*wx+Iyy*wx-2*Ixy*wy+Ixz*wz)+(Ixz*Iyy-Ixy*Iyz)*(Ixz*wx+2*Iyz*wy-(Iyy-Izz)*wz);
% % i=(Ixy^2-Ixx*Iyy)*(Iyz*wx+Ixz*wy)-(Ixy*Ixz-Ixx*Iyz)*(Ixx*wx-Izz*wx+Ixy*wy+2*Ixz*wz)+(Ixz*Iyy-Ixy*Iyz)*(-Ixy*wx-Iyy*wy+Izz*wy-2*Iyz*wz);

% det = (Ixz^2*Iyy - 2*Ixy*Ixz*Iyz + Ixy^2*Izz + Ixx*(Iyz^2 - Iyy*Izz));

% a=(-(Iyz^2 - Iyy*Izz)*(Ixz*wy - Ixy*wz) - (Ixz*Iyy - Ixy*Iyz)*(2*Ixy*wx -Ixx*wy + Iyy*wy + Iyz*wz) - (Ixz*Iyz - Ixy*Izz)*(2*Ixz*wx + Iyz*wy - (Ixx - Izz)*wz))/det;
% b=(-(Ixz*Iyz - Ixy*Izz)*(Iyz*wx - Ixy*wz) - (Ixz*Iyy - Ixy*Iyz)*(-Ixx*wx + Iyy*wx - 2*Ixy*wy + Ixz*wz) - (Iyz^2 - Iyy*Izz)*(Ixz*wx + 2*Iyz*wy - (Iyy - Izz)*wz))/det;
% c=(-(Ixz*Iyy - Ixy*Iyz)*(Iyz*wx + Ixz*wy) - (Ixz*Iyz - Ixy*Izz)*(-Ixx*wx + Izz*wx - Ixy*wy - 2*Ixz*wz) - (Iyz^2 - Iyy*Izz)*(-Ixy*wx - Iyy*wy + Izz*wy - 2*Iyz*wz))/det;
% d=((Ixz*Iyz - Ixy*Izz)*(Ixz*wy - Ixy*wz) + (Ixy*Ixz - Ixx*Iyz)*(2*Ixy*wx - Ixx*wy + Iyy*wy + Iyz*wz) + (Ixz^2 - Ixx*Izz)*(2*Ixz*wx + Iyz*wy - (Ixx - Izz)*wz))/det;
% e=((Ixz^2 - Ixx*Izz)*(Iyz*wx - Ixy*wz) + (Ixy*Ixz - Ixx*Iyz)*(-Ixx*wx + Iyy*wx - 2*Ixy*wy + Ixz*wz) + (Ixz*Iyz - Ixy*Izz)*(Ixz*wx + 2*Iyz*wy - (Iyy - Izz)*wz))/det;
% f=((Ixy*Ixz - Ixx*Iyz)*(Iyz*wx + Ixz*wy) + (Ixz^2 - Ixx*Izz)*(-Ixx*wx + Izz*wx - Ixy*wy - 2*Ixz*wz) + (Ixz*Iyz - Ixy*Izz)*(-Ixy*wx - Iyy*wy + Izz*wy - 2*Iyz*wz))/det;
% g=(-(Ixz*Iyy - Ixy*Iyz)*(Ixz*wy - Ixy*wz) + (-Ixy^2 + Ixx*Iyy)*(2*Ixy*wx - Ixx*wy + Iyy*wy + Iyz*wz) + (Ixy*Ixz - Ixx*Iyz)*(-2*Ixz*wx - Iyz*wy + (Ixx - Izz)*wz))/det;
% h=((Ixy*Ixz - Ixx*Iyz)*(-Iyz*wx + Ixy*wz) + (-Ixy^2 + Ixx*Iyy)*(-Ixx*wx +Iyy*wx - 2*Ixy*wy + Ixz*wz) - (Ixz*Iyy - Ixy*Iyz)*(Ixz*wx + 2*Iyz*wy - (Iyy - Izz)*wz))/det;
% i=((-Ixy^2 + Ixx*Iyy)*(Iyz*wx + Ixz*wy) + (Ixy*Ixz - Ixx*Iyz)*(Ixx*wx - Izz*wx + Ixy*wy + 2*Ixz*wz) - (Ixz*Iyy - Ixy*Iyz)*(-Ixy*wx - Iyy*wy + Izz*wy - 2*Iyz*wz))/det;

% a = 1 + T*a;
% b = T*b;
% c = T*c;
% d = T*d;
% e = 1 + T*e;
% f = T*f;
% g = T*g;
% h = T*h;
% i = 1 + T*i;

% % Each column is process differentiated W.R.T.
% % x,y,z,vx,vy,vz,wx,wy,wz,a1,a2,a3

% F=[1 0 0 T 0 0 0 0 0 0 0 0;
%    0 1 0 0 T 0 0 0 0 0 0 0;
%    0 0 1 0 0 T 0 0 0 0 0 0;
%    0 0 0 1 0 0 0 0 0 0 0 0;
%    0 0 0 0 1 0 0 0 0 0 0 0;
%    0 0 0 0 0 1 0 0 0 0 0 0;
%    0 0 0 0 0 0 1 0 0 T 0 0;
%    0 0 0 0 0 0 0 1 0 0 T 0;
%    0 0 0 0 0 0 0 0 1 0 0 T;
%    0 0 0 0 0 0 0 0 0 a b c;
%    0 0 0 0 0 0 0 0 0 d e f;
%    0 0 0 0 0 0 0 0 0 g h i];