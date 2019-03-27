function [ DCM ] = Quaternion2DCM( quaternion )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

q0 = quaternion(1);
q1 = quaternion(2);
q2 = quaternion(3);
q3 = quaternion(4);

DCM(1,1) = q0^2 + q1^2 - q2^2 - q3^2;
DCM(2,2) = q0^2 - q1^2 + q2^2 - q3^2;
DCM(3,3) = q0^2 - q1^2 - q2^2 + q3^2;

DCM(1,2) = 2*(q1*q2 + q0*q3);
DCM(1,3) = 2*(q1*q3 - q0*q2);
DCM(2,1) = 2*(q1*q2 - q0*q3);
DCM(2,3) = 2*(q2*q3 + q0*q1);
DCM(3,1) = 2*(q1*q3 + q0*q2);
DCM(3,2) = 2*(q2*q3 - q0*q1);

end

