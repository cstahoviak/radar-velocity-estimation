function [ G, V, M ] = odr_getJacobian2D_v2( X, delta, beta, weights, E )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% NOTE: We will use ODRPACK95 notation where the total Jacobian J has block
% components G, V and D:

% J = [G,          V;
%      zeros(n,p), D]

% G - the Jacobian matrix of epsilon wrt/ beta and has no special
% properites
% V - the Jacobian matrix of epsilon wrt/ delta and is a diagonal matrix
% D - the Jacobian matrix of delta wrt/ delta and is a diagonal matrix

% d - error variance ratio sigma_epsilon/sigma_delta

Ntargets = size(X,1);   % X is a row vector of angle values
p = size(beta,1);

% initialize
G = zeros(Ntargets,p);
V = zeros(Ntargets);
M = zeros(Ntargets);

for i=1:Ntargets
    G(i,:) = weights(i)*[cos(X(i) + delta(i)), sin(X(i) + delta(i))];
    V(i,i) = weights(i)*(-beta(1)*sin(X(i) + delta(i)) + ...
        beta(2)*cos(X(i) + delta(i)));
    
    % (ODR-1987 Prop. 2.1)
    w =  V(i,i)^2 / E(i,i);
    M(i,i) = sqrt(1/(1+w));
end

end

