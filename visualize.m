clear all; 
clf; 

raw = load('part1/p1n00'); 
filtered = load('part1/p1a00');

% raw = raw(1:10, :); 
% filtered = filtered(1:10, :); 

markers_body = [
  -1.5   -3.0   -4.5
  -1.5   -3.0    1.5
  -1.5    1.0   -4.5
  -1.5    1.0    1.5
   0.5   -3.0   -4.5
   0.5   -3.0    1.5
   0.5    1.0   -4.5
   0.5    1.0    1.5];

faces = [1 3 4 2; 5 7 8 6; 1 5 6 2; 3 7 8 4]; 
faces = [faces faces(:,1)]; 

x = raw(:,1:3:end); 
y = raw(:,2:3:end); 
z = raw(:,3:3:end); 
xmin = min(x(:))-3; 
xmax = max(x(:))+3; 
ymin = min(y(:))-3; 
ymax = max(y(:))+3; 
zmin = min(z(:))-3; 
zmax = max(z(:))+3; 

figure('visible', 'off'); 

vid_writer = VideoWriter('part1_p1n00.avi');
vid_writer.FrameRate = 24; 
open(vid_writer); 
for i = 1:size(raw,1)
	fprintf('progress: %d/%d\n', i, size(raw,1));
	
	clf;
	v_raw = reshape(raw(i,:), 3, 8)';
	hold on;
	for j = 1:size(faces,1)
		plot3(v_raw(faces(j,:),1), v_raw(faces(j,:),2), v_raw(faces(j,:),3), ...
			'--', 'color', 'r', 'linewidth', 2);
	end	
	scatter3(v_raw(:,1), v_raw(:,2), v_raw(:,3), 100, 'ro', 'filled');
	R = quat2dcm(filtered(i,7:10)); 
	markers_body_rotated = (R * markers_body')'; 
	COM = filtered(i,1:3); 
	v_fil = bsxfun(@plus, markers_body_rotated, COM); 
	for j = 1:size(faces,1)
		plot3(v_fil(faces(j,:),1), v_fil(faces(j,:),2), v_fil(faces(j,:),3), ...
			'-', 'color', 'b', 'linewidth', 4);
	end	
	scatter3(v_fil(:,1), v_fil(:,2), v_fil(:,3), 100, 'bo', 'filled');
	hold off;
	% keyboard;
	xlim([xmin xmax]); 
	ylim([ymin ymax]); 
	zlim([zmin zmax]); 
	view(3); 
	grid on; 
	drawnow; 

	fig = getframe(gcf);
	writeVideo(vid_writer, fig);
end	
close(vid_writer); 
