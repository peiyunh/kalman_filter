function [ marker_error, H ] = predict_markers( quaternion, state_estimate, index, markersn );
%
%
global n_m n_x markers_body

%% same as extracted from state 
R = quat2dcm(quaternion'); 
markers_body_rotated = (R * markers_body')'; 

% NOTE: verified below
% ax=state_estimate(7);
% ay=state_estimate(8);
% az=state_estimate(9);
% sx=sin(ax); cx=cos(ax);
% sy=sin(ay); cy=cos(ay);
% sz=sin(az); cz=cos(az);
% R2 = [cy*cz         , -cy*sz         ,  sy;
%      cx*sz+cz*sx*sy,  cx*cz-sx*sy*sz, -cy*sx;
%      sx*sz-cx*cz*sy,  cx*sy*sz+cz*sx,  cx*cy];
% if any(abs(R(:)-R2(:))>1e-3)
%   keyboard;
% end 

for i = 1:n_m
 for j = 1:3
  ind = (i-1)*3 + j;
  marker_error(ind) = markers_body_rotated(i,j) + state_estimate(j) - markersn(index,ind);
 end
end
% make it vertical
marker_error = marker_error';

%% compute measurement jacobian   
ax = state_estimate(7); sx = sin(ax); cx = cos(ax); 
ay = state_estimate(8); sy = sin(ay); cy = cos(ay); 
az = state_estimate(9); sz = sin(az); cz = cos(az); 

%% NOTE: we used symoblic derivative to compute the jacobian of H 
% see compute_jacobian.m for more detail
H = zeros(3*n_m, n_x); 
for i = 1:n_m
  x = markers_body(i,1); 
  y = markers_body(i,2); 
  z = markers_body(i,3);
  dp = [1 0 0 0 0 0
        0 1 0 0 0 0 
        0 0 1 0 0 0]; 
  dx = [   0
         - x*(sx*sz - cx*cz*sy) - y*(cz*sx + cx*sy*sz) - z*cx*cy
           x*(cx*sz + cz*sx*sy) + y*(cx*cz - sx*sy*sz) - z*cy*sx]; 
  dy = [   z*cy - x*cz*sy + y*sy*sz
           z*sx*sy + x*cy*cz*sx - y*cy*sx*sz
           y*cx*cy*sz - x*cx*cy*cz - z*cx*sy]; 
  dz = [ - y*cy*cz - x*cy*sz
           x*(cx*cz - sx*sy*sz) - y*(cx*sz + cz*sx*sy)
           x*(cz*sx + cx*sy*sz) - y*(sx*sz - cx*cz*sy)]; 
  dw = [0 0 0
        0 0 0 
        0 0 0]; 
  H((i-1)*3+[1:3], :) = [dp dx dy dz dw];
end

occ_idx = find(markersn(index,:)==1e10);
if ~isempty(occ_idx)
  H(occ_idx,:)=0;
  marker_error(max(occ_idx)/3)=0;
end