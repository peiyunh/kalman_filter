function [ quaternion, state_estimate] = hack_init_estimates( markersn )

global n_m T 

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

% NOTE handle occluded cases 
occ_idx = find(markersn(1,:)==1e10);
if ~isempty(occ_idx) 
	fprintf('Found %d marker occluded in the first measurement.\n', numel(occ_idx)/3);
	A(occ_idx,:) = []; 
	b(occ_idx) = []; 
end	

% actually do regression
b = b';
p = A\b;

% initialize COM velocity with centroid velocity (approximately)
markersn = reshape(markersn', 3, 8, []);
markersn(markersn==1e10) = nan; 
markersn = squeeze(nanmean(markersn, 2)); 
v = (markersn(:,end) - markersn(:,1)) / ( (size(markersn,2)-1)*T ) ; 

quaternion = [1 0 0 0]';
state_estimate = [ p(1) p(2) p(3) v(1) v(2) v(3) 0 0 0 0 0 0 ]';