function compute_jacobian()

I_body = diag([1.571428571428571, 5.362637362637362, 7.065934065934067]);

syms ax ay az 
sx = sin(ax); cx = cos(ax); 
sy = sin(ay); cy = cos(ay); 
sz = sin(az); cz = cos(az); 
R = [cy*cz         ,  -cy*sz        ,  sy;
     cx*sz+cz*sx*sy,  cx*cz-sx*sy*sz,  -cy*sx;
     sx*sz-cx*cz*sy,  cx*sy*sz+cz*sx,  cx*cy];

% syms wx wy wz 
% I_world = R * I_body * R';  
% w = [wx; wy; wz;]
% wdot = -inv(I_world) * cross(w, I_world*w);

%% for H (measurement)
syms x y z 
rxyz = R * [x; y; z]; 
Hx = diff(rxyz, ax) 
Hy = diff(rxyz, ay)
Hz = diff(rxyz, az)

%% for F (process)
%% NOTE: since we cannot easily solve this with symbolic derivatives 
%%       we solve it numerically 
% syms T 
% new_w = w + wdot * T; 
% diff(new_w, ax)
% diff(new_w, ay)
% diff(new_w, az)