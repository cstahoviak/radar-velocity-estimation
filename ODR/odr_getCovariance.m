function [ cov_beta ] = odr_getCovariance( G, V, D, eps, delta, weights, d )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

n = size(G,1);      % number of targets in the scan
p = size(G,2);      % dimension of the model parameters

% form complete Jacobian matrix
J = [G,         V; 
     zeros(n,p) D];

% form total weighted residual vector: g = [eps, delta]'
g = [weights.*eps; weights.*d.*delta];

% compute total weighted covariance matrix
cov = 1/(n-p)*(g'*g)*inv(J'*J);

% return covariance of model parameters only
cov_beta = cov(1:p,1:p);

end

