function [ fig_h, ax_h ] = createVelocityEstimationPlots( )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% ground truth body-frame velocities
fh1 = figure(1);
axh1 = gca; hold(axh1, 'on');
title('Body-Frame Vehicle Velocities - Vicon System','Interpreter','latex');
xlabel('time [s]','Interpreter','latex');
ylabel('velocity [m/s]','Interpreter','latex');

% forward velocity (v_x) estimate vs. truth - OLD METHOD
fh2 = figure(2);
axh2 = gca; hold(axh2, 'on');
title({'Doppler Velocity Estimation - Old Method',...
    'Mean and Maximum Estimation'},'Interpreter','latex');
xlabel('time [s]','Interpreter','latex');
ylabel('forward velocity [m/s]','Interpreter','latex');

% 'brute force' velocity estimate
fh3 = figure(3);
axh3_1 = subplot(2,1,1); hold(axh3_1, 'on'); grid;
axh3_2 = subplot(2,1,2); hold(axh3_2, 'on'); grid;
title(axh3_1,'Brute Force Velocity Estimate','Interpreter','latex');
xlabel(axh3_2,'time [s]','Interpreter','latex');
ylabel(axh3_1,'$v_x$ [m/s]','Interpreter','latex');
ylabel(axh3_2,'$v_y$ [m/s]','Interpreter','latex');

% MLESAC velocity estimate
fh4 = figure(4);
axh4_1 = subplot(2,1,1); hold(axh4_1, 'on'); grid;
axh4_2 = subplot(2,1,2); hold(axh4_2, 'on'); grid;
title(axh4_1,'MLESAC Velocity Estimate','Interpreter','latex');
xlabel(axh4_2,'time [s]','Interpreter','latex');
ylabel(axh4_1,'$v_x$ [m/s]','Interpreter','latex');
ylabel(axh4_2,'$v_y$ [m/s]','Interpreter','latex');

% MLESAC inlier set - 'brute force' velocity estimate
fh5 = figure(5);
axh5_1 = subplot(2,1,1); hold(axh5_1, 'on'); grid;
axh5_2 = subplot(2,1,2); hold(axh5_2, 'on'); grid;
title(axh5_1,'Brute Force Velocity Estimate - MLESAC Inlier Set','Interpreter','latex');
xlabel(axh5_2,'time [s]','Interpreter','latex');
ylabel(axh5_1,'$v_x$ [m/s]','Interpreter','latex');
ylabel(axh5_2,'$v_y$ [m/s]','Interpreter','latex');

% ground truth euler angles
fh6 = figure(6);
axh6 = gca; hold(axh6, 'on'); grid;
title('Euler Angles - Vicon System','Interpreter','latex');
xlabel('time [s]','Interpreter','latex');
ylabel('angle [deg]','Interpreter','latex');

fig_h = [fh1; fh2; fh3; fh4; fh5; fh6];
ax_h  = [axh1; axh2; axh3_1; axh3_2; axh4_1; axh4_2; axh5_1; axh5_2; axh6];


end