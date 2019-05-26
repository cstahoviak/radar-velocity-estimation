function [ deriv ] = central_diff( X, h )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

N = size(X,1);
deriv = NaN*ones(N-2,1);

for i=1:N-2
    deriv(i) = (X(i+2) - X(i))/(2*h);
end

