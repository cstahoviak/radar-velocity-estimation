function [ fig_h, ax_h ] = createVelocityEstimationPlots_3D( )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% Vicon pose data - /vrpn_client_node/<object>/pose/position
fh1 = figure(1);
axh1_1 = subplot(3,1,1); hold(axh1_1, 'on'); grid;
axh1_2 = subplot(3,1,2); hold(axh1_2, 'on'); grid;
axh1_3 = subplot(3,1,3); hold(axh1_3, 'on'); grid;
title(axh1_1,'Ground Truth Position','Interpreter','latex');
ylabel(axh1_1,'$x$ [m]','Interpreter','latex');
ylabel(axh1_2,'$y$ [m]','Interpreter','latex');
ylabel(axh1_3,'$z$ [m]','Interpreter','latex');
xlabel(axh1_3,'time [s]','Interpreter','latex');

% Vicon body-frame velocities:
% 3. VRPN topic - /vrpn_client_node/<object>/twist
% 2. central-diff + lowpass filter method
% 3. central-diff + lowpass filter + moving average filter method
fh2 = figure(2);
axh2_1 = subplot(3,1,1); hold(axh2_1, 'on'); grid;
axh2_2 = subplot(3,1,2); hold(axh2_2, 'on'); grid;
axh2_3 = subplot(3,1,3); hold(axh2_3, 'on'); grid;
title(axh2_1,'Ground Truth Body-Frame Velocity Components --','Interpreter','latex');
ylabel(axh2_1,'$v_x$ [m/s]','Interpreter','latex');
ylabel(axh2_2,'$v_y$ [m/s]','Interpreter','latex');
ylabel(axh2_3,'$v_z$ [m/s]','Interpreter','latex');
xlabel(axh2_3,'time [s]','Interpreter','latex');

% ground truth euler angles - /vrpn_client_node/<object>/pose/orientation
fh3 = figure(3);
axh3 = gca; hold(axh3, 'on'); grid;
title('Euler Angles - Vicon System','Interpreter','latex');
xlabel('time [s]','Interpreter','latex');
ylabel('angle [deg]','Interpreter','latex');

% MLESAC ego-velocity estimate + groundtruth
fh4 = figure(4);
axh4_1 = subplot(3,1,1); hold(axh4_1, 'on'); grid;
axh4_2 = subplot(3,1,2); hold(axh4_2, 'on'); grid;
axh4_3 = subplot(3,1,3); hold(axh4_3, 'on'); grid;
title(axh4_1,'MLESAC Ego-Velocity Estimate','Interpreter','latex');
ylabel(axh4_1,'$v_x$ [m/s]','Interpreter','latex');
ylabel(axh4_2,'$v_y$ [m/s]','Interpreter','latex');
ylabel(axh4_3,'$v_z$ [m/s]','Interpreter','latex');
xlabel(axh4_3,'time [s]','Interpreter','latex');

% LSQNONLIN ego-velocity estimate + groundtruth
fh5 = figure(5);
axh5_1 = subplot(3,1,1); hold(axh5_1, 'on'); grid;
axh5_2 = subplot(3,1,2); hold(axh5_2, 'on'); grid;
axh5_3 = subplot(3,1,3); hold(axh5_3, 'on'); grid;
title(axh5_1,'LSQNONLIN Ego-Velocity Estimate','Interpreter','latex');
ylabel(axh5_1,'$v_x$ [m/s]','Interpreter','latex');
ylabel(axh5_2,'$v_y$ [m/s]','Interpreter','latex');
ylabel(axh5_3,'$v_z$ [m/s]','Interpreter','latex');
xlabel(axh5_3,'time [s]','Interpreter','latex');

% Const. Weight ODR_v5 ego-velocity estimate + groundtruth
fh6 = figure(6);
axh6_1 = subplot(3,1,1); hold(axh6_1, 'on'); grid;
axh6_2 = subplot(3,1,2); hold(axh6_2, 'on'); grid;
axh6_3 = subplot(3,1,3); hold(axh6_3, 'on'); grid;
title(axh6_1,'Constant Weight ODR\_v5 Ego-Velocity Estimate','Interpreter','latex');
ylabel(axh6_1,'$v_x$ [m/s]','Interpreter','latex');
ylabel(axh6_2,'$v_y$ [m/s]','Interpreter','latex');
ylabel(axh6_3,'$v_z$ [m/s]','Interpreter','latex');
xlabel(axh6_3,'time [s]','Interpreter','latex');

% Weighted ODR_v5 ego-velocity estimate + groundtruth
fh7 = figure(7);
axh7_1 = subplot(3,1,1); hold(axh7_1, 'on'); grid;
axh7_2 = subplot(3,1,2); hold(axh7_2, 'on'); grid;
axh7_3 = subplot(3,1,3); hold(axh7_3, 'on'); grid;
title(axh7_1,'Weighted ODR\_v5 Ego-Velocity Estimate','Interpreter','latex');
ylabel(axh7_1,'$v_x$ [m/s]','Interpreter','latex');
ylabel(axh7_2,'$v_y$ [m/s]','Interpreter','latex');
ylabel(axh7_3,'$v_z$ [m/s]','Interpreter','latex');
xlabel(axh7_3,'time [s]','Interpreter','latex');

% Weighted ODR_v5 + LSQNONLIN ego-velocity estimate + groundtruth
fh8 = figure(8);
axh8_1 = subplot(3,1,1); hold(axh8_1, 'on'); grid;
axh8_2 = subplot(3,1,2); hold(axh8_2, 'on'); grid;
axh8_3 = subplot(3,1,3); hold(axh8_3, 'on'); grid;
title(axh8_1,{'Ego-Velocity Estimate','Constand Weight ODR\_v5 vs. LSQNONLIN'}, ...
    'Interpreter','latex');
ylabel(axh8_1,'$v_x$ [m/s]','Interpreter','latex');
ylabel(axh8_2,'$v_y$ [m/s]','Interpreter','latex');
ylabel(axh8_3,'$v_z$ [m/s]','Interpreter','latex');
xlabel(axh8_3,'time [s]','Interpreter','latex');

% Weighted ODR_v5 + covariance bounds centered on estimate
fh9 = figure(9);
axh9_1 = subplot(3,1,1); hold(axh9_1, 'on'); grid;
axh9_2 = subplot(3,1,2); hold(axh9_2, 'on'); grid;
axh9_3 = subplot(3,1,3); hold(axh9_3, 'on'); grid;
title(axh9_1,'Weighted ODR\_v5 + Covariance Bounds','Interpreter','latex');
ylabel(axh9_1,'$v_x$ [m/s]','Interpreter','latex');
ylabel(axh9_2,'$v_y$ [m/s]','Interpreter','latex');
ylabel(axh9_3,'$v_z$ [m/s]','Interpreter','latex');
xlabel(axh9_2,'time [s]','Interpreter','latex');

% Weighted ODR_v5 + covariance bounds centered at zero
fh10 = figure(10);
axh10_1 = subplot(3,1,1); hold(axh10_1, 'on'); grid;
axh10_2 = subplot(3,1,2); hold(axh10_2, 'on'); grid;
axh10_3 = subplot(3,1,3); hold(axh10_3, 'on'); grid;
title(axh10_1,'Weighted ODR\_v5 + Covariance Bounds','Interpreter','latex');
ylabel(axh10_1,'$v_x$ [m/s]','Interpreter','latex');
ylabel(axh10_2,'$v_y$ [m/s]','Interpreter','latex');
ylabel(axh10_3,'$v_z$ [m/s]','Interpreter','latex');
xlabel(axh10_2,'time [s]','Interpreter','latex');

% concatenate figure and axes handles
fig_h = [fh1; fh2; fh3; fh4; fh5; fh6; fh7; fh8; fh9; fh10];
ax_h  = [axh1_1; axh1_2; axh1_3; axh2_1; axh2_2; axh2_3; axh3; ...
         axh4_1; axh4_2; axh4_3; axh5_1; axh5_2; axh5_3; ...
         axh6_1; axh6_2; axh6_3; axh7_1; axh7_2; axh7_3; ...
         axh8_1; axh8_2; axh8_3; axh9_1; axh9_2; axh9_3; ...
         axh10_1; axh10_2; axh10_3];

end