function [ gaussian_dist ] = gauss_dist( distribution, variance)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
gaussian_dist = (1/(sqrt(2*pi*variance^2)))*exp(-(1/2)*(distribution^2/variance^2));

end

