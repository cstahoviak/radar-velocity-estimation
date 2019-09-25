function [ G, V, M ] = odr_getJacobian3D_v2( X, delta, beta, weights, E )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% NOTE: We will use ODRPACK95 notation where the total Jacobian J has block
% components G, V and D:

% J = [G,            V;
%      zeros(n*m,p), D]

% G - Jacobian matrix of epsilon wrt/ beta and has no special properites,
%     (n x p)
% V - Jacobian matrix of epsilon wrt/ delta and is a diagonal matrix,
%     (n x nm)
% D - Jacobian matrix of delta wrt/ delta and is a diagonal matrix,
%     (nm x nm)

Ntargets = size(X,1)/2;
p = size(beta,1);
m = size(delta,1)/Ntargets;

theta = X(1:Ntargets);
phi   = X(Ntargets+1:2*Ntargets);

% "un-interleave" delta vector into (Ntargets x m) matrix
delta = reshape(delta,[m,Ntargets])';
delta_theta = delta(:,1);
delta_phi   = delta(:,2);

% defined to simplify the following calculations
x1 = theta + delta_theta;
x2 = phi + delta_phi;

% initialize
G = zeros(Ntargets,p);
V = zeros(Ntargets,Ntargets*m);
M = zeros(Ntargets);

for i=1:Ntargets
    G(i,1:p) = weights(i)*[cos(x1(i))*cos(x2(i)), sin(x1(i))*cos(x2(i)), sin(x2(i))];
          
    V(i,2*i-1:2*i) = weights(i)* ...
        [-beta(1)*sin(x1(i))*cos(x2(i)) + beta(2)*cos(x1(i))*cos(x2(i)), ...
         -beta(1)*cos(x1(i))*sin(x2(i)) - beta(2)*sin(x1(i))*sin(x2(i)) + ...
          beta(3)*cos(x2(i))];
      
    % (ODR-1987 Prop. 2.1)
    w = V(i,2*i-1)^2 / E(2*i-1,2*i-1) + V(i,2*i)^2 / E(2*i,2*i);
    M(i,i) = sqrt(1/(1+w));
end

end

