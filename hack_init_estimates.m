function [ quaternion, state_estimate] = hack_init_estimates( markersn )

global n_m markers_body T 

% use linear regression to estimate COM location
A = zeros( n_m*3, 3 );
b(n_m*3) = 0;

% load up matrices: A*p = b
for i = 1:n_m
 for j = 1:3
  row = (i-1)*3 + 1;
  A(row+j-1,j) = 1;
  b(row+j-1) = markersn( 1, row+j-1 );
 end
end

% NOTE: uncomment to handle the occlusion case
% [~, i] = find(markersn(1,:) == 1e10); 
% A(i,:) = []; 
% b(i) = []; 

% actually do regression
b = b';
p = A\b;
% check errors (residuals)
% A*p - b

% initialize COM velocity with centroid velocity (approximately)
markersn = squeeze(reshape(markersn', 3, 8, []));
v = (markersn(:,end) - markersn(:,1)) / ( (size(markersn,3)-1)*T ) ; 

quaternion = [1 0 0 0]';
state_estimate = [ p(1) p(2) p(3) v(1) v(2) v(3) 0 0 0 0 0 0 ]';