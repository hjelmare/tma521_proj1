function [ d ] = CalcSubGradient(x, n, k)
%CalcSubGradient calculates the subgradient
%   NOTE that the innermost for-loop uses the same argument as in
%   CalculateSubProblem (compare this func with pdf).

d = zeros(n,1);
%nbrOfAcceptedPaths = length(okcom);
for i = 1:n
    temp = 0;
    for l = 1:k
        for j = 1:n
            temp = temp + x(j, i, l);
        end
    end
    d(i) = 1 - temp;
            
end

