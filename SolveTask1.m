% Initial setup of some stuff
pi = startPi*ones(n,1);
H = zeros(1,maxIterations); % Dual value at each iteration (for plotting)
nPaths = zeros(1,maxIterations); % Number of accepted paths ---||---

nLambda = 0;
hOld=inf;
for nIteration = 1:maxIterations
    % x is the vector that describes the paths, okcom is the paths that were accepted
    [nl, okcom, x] = LagrangianSubProblem(dimX, dimY, com, k, pi);
    % h is the value of the dual-problem value at this iteration
    h = CalculateSubProblem(pi, okcom, nl); 
   
    d = CalcSubGradient(x, n, k); %d is the subgradient
   
    s = CalcStepLength(lambda, h, d); %s is the step length
   
    pi = UpdatePi(pi, s, d, n);
   
    % Updating lambda
    if hOld > h
        hOld = h;
        nLambda = 0;
    end
    if nLambda == lambdaIterations  % If no improvement for x iterations
        lambda = lambda * 0.5;      
        nLambda = 0;
    end
    nLambda = nLambda + 1;
   
    H(nIteration) = h;
end

% Plot stuff
storlek = 14;
plot(H)
title('Dual-problem value', 'FontSize', storlek)
xlabel('iteration number', 'FontSize', storlek)
ylabel('value', 'FontSize', storlek)
disp(['Final value of dual function is ' num2str(H(end)) ])

shift = 25;
figure
visagrid(dimX,dimY,nl,com,pi,shift)
title('Path of the different pairs with no heuristic', 'FontSize', storlek)
