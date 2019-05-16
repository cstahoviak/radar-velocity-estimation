function [ twist_linear_body ] = ...
    getBodyFrameVelocities( twist_linear, orientation );
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

NScans = size(twist_linear,1);

% initialize
twist_linear_body = zeros(NScans,3);

for i=1:NScans
    
    % get Direction Cosine Matrix (DCM)
    DCM = quaternion2DCM( orientation(i,:) );
    
    % transform from inertial frame to body frame
    twist_linear_body(i,:) = (DCM*twist_linear(i,:)')';
end

end

