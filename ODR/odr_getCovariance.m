function [ cov_beta ] = odr_getCovariance( G, V, D, eps, delta, weights, d )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

n = size(G,1);      % number of targets in the scan
p = size(G,2);      % dimension of the model parameters
m = size(d,1);      % dimension of 'explanatory variable' vector

% form complete Jacobian matrix
J = [G,           V; 
     zeros(n*m,p) D];

% form total weighted residual vector: g = [eps, delta]'
if p == 2 && m == 1
    g = [weights.*eps; d*weights.*delta];
elseif p == 3 && m == 2
    g = [weights.*eps; ...
        d(1)*(weights.*delta(1:n)); ...
        d(2)*(weights.*delta(n+1:2*n))];
else
    error('p and m size mismatch')
end

% compute total weighted covariance matrix
cov = 1/(n-p)*(g'*g)*inv(J'*J);

% return covariance of model parameters only
cov_beta = cov(1:p,1:p);

end

