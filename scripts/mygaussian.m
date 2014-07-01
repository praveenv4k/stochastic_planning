function [ y ] = mygaussian(x, mu, sigma)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

 
y = (1/sqrt(2*pi))*exp(-((x-mu).^2)/2*sigma);

end