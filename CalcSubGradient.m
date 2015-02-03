function [ d ] = CalcSubGradient(x, n)
%CalcSubGradient calculates the subgradient
%   NOTE that the innermost for-loop uses the same argument as in
%   CalculateSubProblem (compare this func with pdf).

d = zeros(n,1);
%nbrOfAcceptedPaths = length(okcom);
for i = 1:n
    temp = sum( x(:, i, :) );
    d(i) = 1 - sum(temp);
            
end

