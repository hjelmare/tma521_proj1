function [ h ] = CalculateSubProblem(x, pi, n, okcom)
%Calculates the value of the subproblem given the optimized paths
nbrOfAcceptedPaths = length(okcom);

h = sum(pi);

%the outmost of these for -loops goes from 1 to the number of accepted
%paths while in the pdf it goes from 1 to the number of nodes. The reason
%to why we have done this is that x_{s_l t_l l}=0 and x_{jil} = 0 when we
%did not accept the path for pair l. This means that we do not have to
%calculate these in the for-loop.
temp = 0;
for l = 1:nbrOfAcceptedPaths
    for i = 1:n
        for j = 1:n
            temp = temp * pi(i)*x(j, i, okcom(l));
        end
    end
    
    h = h + 1 - temp;
end

