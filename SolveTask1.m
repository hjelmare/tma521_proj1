% Initial setup of some stuff
pi = startPi*ones(n,1);
H = zeros(1,maxIterations); % Dual value at each iteration (for plotting)
primalValue = zeros(1,maxIterations); % Primal value (for plotting)
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
    
    if(mod(nIteration,1)==0)
        tempCom = com(okcom,:);
        if ~isempty(tempCom)
            primalValue(nIteration) = Heuristic(nl, tempCom, dimX, dimY);
        else
            %disp('den var tom')
            primalValue(nIteration) = 0;
        end
    end
end

% Plot stuff
storlek = 14;
plot(H)
hold on
plot(primalValue, 'r')
title('Dual-problem value', 'FontSize', storlek)
xlabel('iteration number', 'FontSize', storlek)
ylabel('value', 'FontSize', storlek)
legend('Dual problem value', 'Primal problem value', 'FontSize', storlek)
disp(['Final value of dual function is ' num2str(H(end)) ])

shift = 25;
figure
visagrid(dimX,dimY,nl,com,pi,shift)
title('Path of the different pairs with no heuristic', 'FontSize', storlek)

%% Calculate numbers to the Heuristic
% These numbers are calculated from the 100 LAST primal solutions

differentSolutions = unique(primalValue);
nbrOfSolutions = length(differentSolutions);
result = zeros(nbrOfSolutions, 2);
for i = 1:nbrOfSolutions
    result(i,1) = differentSolutions(i);
    result(i,2) = sum(primalValue(end-99:end) == differentSolutions(i))/100; %In procent
end

figure
bar(differentSolutions, result(:,2))
title('Result from heuristic', 'FontSize', storlek)
xlabel('primal problem value', 'FontSize', storlek)
ylabel('number of primal problem values', 'FontSize', storlek)

