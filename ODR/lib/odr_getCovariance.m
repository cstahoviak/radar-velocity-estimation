function [ cov_beta ] = odr_getCovariance( Gbar, D, eps, delta, weights )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

n = size(Gbar,1);       % number of targets in the scan
p = size(Gbar,2);       % dimension of the model parameters
m = size(delta,1)/n;    % dimension of 'explanatory variable' vector

% form complete residual vector, g
g = [eps; delta];

% residual weighting matrix, Omega
W = diag(weights.^2);
Omega = [W,           zeros(n,n*m);
         zeros(n*m,n) D^2];

% compute total weighted covariance matrix
cov = 1/(n-p)*(g'*Omega*g)*inv(Gbar'*Gbar);

% return covariance of model parameters only
cov_beta = cov(1:p,1:p);

end

