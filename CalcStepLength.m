function [ s ] = CalcStepLength(lambda, h, d)
%Calculates the step length

temp = norm(d)^2;

s = lambda*h/temp;


end

