function plotit(states_m, states_p, quaternions_m, quaternions_p, variances_m, variances_p, gains, data_file)

%% tracking information 
figure(1); 

% x y z 
subplot(2,1,1); 
plot(states_m(:,1:3), '-');
% hold on; 
% plot(states_p(:,1:3), '--');
% hold off;
% legend({'x^m', 'y^m', 'z^m', 'x^p', 'y^p', 'z^p'}) ; 
legend({'x^m', 'y^m', 'z^m'}) ; 
title('Spatial position track'); 
grid on;

% vx vy vz 
subplot(2,1,2); 
plot(states_m(:,4:6),'-');
% hold on; 
% plot(states_p(:,4:6),'--');
% hold off;
% legend({'v_x^m', 'v_y^m', 'v_z^m', 'v_x^p', 'v_y^p', 'v_z^p'}) ; 
legend({'v_x^m', 'v_y^m', 'v_z^m'}); 
title('Spatial velocity track'); 
grid on;

print('-dpdf', [data_file '_spatial_track.pdf']);

%% 
figure(2);

% ax ay az 
subplot(3,1,1); 
plot(states_m(:,7:9),'-');
% hold on; 
% plot(states_p(:,7:9),'--');
% hold off;
% legend({'\theta_x^m', '\theta_y^m', '\theta_z^m', '\theta_x^p', '\theta_y^p', '\theta_z^p'}) ; 
legend({'\theta_x^m', '\theta_y^m', '\theta_z^m'});
title('Angular position track'); 
grid on;

% wx wy wz 
subplot(3,1,2); 
plot(states_m(:,10:12),'-');
% hold on; 
% plot(states_p(:,10:12),'--');
% hold off;
% legend({'\omega_x^m', '\omega_y^m', '\omega_z^m', '\omega_x^p', '\omega_y^p', '\omega_z^p'}) ; 
legend({'\omega_x^m', '\omega_y^m', '\omega_z^m'}) ; 
title('Angular velocity track'); 
grid on;

% q0 q1 q2 q3 
subplot(3,1,3); 
plot(quaternions_m, '-'); 
% hold on;
% plot(quaternions_p, '--'); 
% hold off;
% legend({'q_0^m', 'q_1^m', 'q_2^m', 'q_3^m', 'q_0^p', 'q_1^p', 'q_2^p', 'q_3^p'}); 
legend({'q_0^m', 'q_1^m', 'q_2^m', 'q_3^m'}); 
title('Quaternion track'); 
grid on;

print('-dpdf', [data_file '_angular_track.pdf']);

%% variance information 
figure(3); 

% x y z 
subplot(2,1,1); 
plot(variances_m(:,1:3), '-');
% hold on; 
% plot(variances_p(:,1:3), '--');
% hold off;
% legend({'var^m(x)', 'var^m(y)', 'var^m(z)', 'var^p(x)', 'var^p(y)', 'var^p(z)'}) ; 
legend({'var^m(x)', 'var^m(y)', 'var^m(z)'});
title('Spatial position variance'); 
grid on;

% vx vy vz 
subplot(2,1,2); 
plot(variances_m(:,4:6), '-');
% hold on; 
% plot(variances_p(:,4:6), '--');
% hold off;
% legend({'var^m(v_x)', 'var^m(v_y)', 'var^m(v_z)', 'var^p(v_x)', 'var^p(v_y)', 'var^p(v_z)'}) ; 
legend({'var^m(v_x)', 'var^m(v_y)', 'var^m(v_z)'});
title('Spatial velocity variance'); 
grid on;

print('-dpdf', [data_file '_spatial_var.pdf']);

%% 
figure(4); 

% ax ay az 
subplot(2,1,1); 
plot(variances_m(:,7:9), '-');
% hold on;
% plot(variances_p(:,7:9), '--');
% hold off;
% legend({'var^m(\theta_x)', 'var^m(\theta_y)', 'var^m(\theta_z)', 'var^p(\theta_x)', 'var^p(\theta_y)', 'var^p(\theta_z)'}) ; 
legend({'var^m(\theta_x)', 'var^m(\theta_y)', 'var^m(\theta_z)'});
title('Angular position variance'); 
grid on;

% wx wy wz 
subplot(2,1,2); 
plot(variances_m(:,10:12), '-');
% hold on;
% plot(variances_p(:,10:12), '--');
% hold off;
% legend({'var^m(\omega_x)', 'var^m(\omega_y)', 'var^m(\omega_z)', 'var^p(\omega_x)', 'var^p(\omega_y)', 'var^p(\omega_z)'}) ; 
legend({'var^m(\omega_x)', 'var^m(\omega_y)', 'var^m(\omega_z)'});
title('Angular velocity variance'); 
grid on;

print('-dpdf', [data_file '_angular_var.pdf']);

%% kalman gains 
figure(5); 
plot(gains);
legend({'x', 'y', 'z'}); 
title('Kalman Gains'); 
grid on;
print('-dpdf', [data_file '_kalman_gain.pdf']);

%% marker positions
if size(states_m, 2) > 12 
	figure(6);
	global n_m n_x
	for i=1:n_m
		subplot(4,2,i); 
		plot(states_m(:, 12+3*(i-1)+[1:3]));
		legend({'x^m', 'y^m', 'z^m'});
		grid on; 
		title(sprintf('Marker #%d spatial position track', i)); 
	end	
	print('-dpdf', [data_file '_marker_body.pdf']); 
end