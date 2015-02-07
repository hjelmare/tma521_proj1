%% Given dimensions and contact paris, this program calculates the maximum
% numbers of pairs that can be made.
clear all
clc
close all
addpath(pwd);

% Select which problem you want to try and solve
  p6
% p10
% p11

n = dimX*dimY*2;    % Number of nodes

% Some settings that one might want to change
maxIterations = 1000;
lambda = 1.99999999;
lambdaFactor = 0.95;    % Reduces lambda ...
lambdaIterations = 10;  % ... every x iterations
startPi = 1/n;          % Initial value for pi (the cost)

% Just preparing things for later
pi = startPi*ones(n,1);
H = zeros(1,maxIterations); % Dual value at each iteration (for plotting)
nPaths = zeros(1,maxIterations); % Number of accepted paths ---||---

for nIteration = 1:maxIterations
    % x is the vector that describes the paths, okcom is the paths that were accepted
    [nl, okcom, x] = LagrangianSubProblem(dimX, dimY, com, k, pi);
    % h is the value of the dual-problem value at this iteration
    h = CalculateSubProblem(x, pi, n, okcom); 
   
    d = CalcSubGradient(x, n); %d is the subgradient
   
    s = CalcStepLength(lambda, h, d); %s is the step length
   
    pi = UpdatePi(pi, s, d, n);
   
    %updates lambda
    if mod(nIteration, lambdaIterations) == 0
        lambda = lambda*lambdaFactor;
    end
   
    H(nIteration) = h;
    nPaths(nIteration) = length(okcom); %KANSKE TA BORT DENNA, STOG INTE ATT MAN SKULLE HA DEN
                                             %MEN KANSKE VILL HA BILD I
                                             %RAPPORT?
   
end

storlek = 14;
plot(H)
title('Dual-problem value', 'FontSize', storlek)
xlabel('iteration number', 'FontSize', storlek)
ylabel('value', 'FontSize', storlek)
disp(['Final value of dual function is ' num2str(H(end)) ])
disp([num2str(nPaths(end)) ' out of ' num2str(k) ' desired paths were accepted'])

shift = 25;
figure
visagrid(dimX,dimY,nl,com,pi,shift)
title('Path of the different pairs with no heuristic', 'FontSize', storlek)


%% Heuristic

% Settings
maxIterations = 50; % Max nr of attempts at fixing collisions

% And here we go...
reallyHighCost = 1e6; % Penalty used to "block" nodes that are already used
pi = (1/n)*ones(n,1);
piTemp = pi;
piTemp(com) = reallyHighCost; % Block start and end nodes

% Calculate route costs, figure out which parts of nl belong to which pair
[routeIndices, routeCost] = UpdateRouteInfo(k, nl, com, piTemp);
% (using piTemp means that all routes get 2x penalty for start/end nodes)

% Next, we want to penalize routes involved in collisions
% Find all nodes that have more than one route passing through
collisionNodes = FindCollisionNodes(nl);
collisionNodes(ismember(collisionNodes,com)) = []; % Ignore start/end nodes
for i = 1:length(collisionNodes)
     % Identify which routes are involved
    collidingRoutes = routeIndices(find(nl == collisionNodes(i)));
    % then find the most expensive one
    mostExpensiveRoute = sort(routeCost(collidingRoutes), 'descend');
    mostExpensiveRoute = find(routeCost == mostExpensiveRoute(1));
    % and put a penalty on it
    routeCost(mostExpensiveRoute) = routeCost(mostExpensiveRoute) + reallyHighCost;
end

% Every route has 2x penalty from start/end nodes, but 3x penalty is a sign
% of being the more expensive route involved in a collision, so anything
% with 3x penalty probably needs to be changed
pairsToChange = find(routeCost > 3*reallyHighCost);
oldNl = nl; %used to check if something has changed in the while-loop.

sameRoutes = 0;
iIteration =1;
while ~isempty(pairsToChange)   % Stop when there are no more collisions
    % Start looking for the worst route (most collisions, highest cost)
    [sortedCosts, sortedRoutes] = sort(routeCost(pairsToChange),'descend');
    
    % klura ut vilken rutt vi ska ändra på
    %collidingRoutes = routeIndices( ismember(nl, collisionNodes) );
    %collidingRoutes = unique( collidingRoutes );
    
    changeRoute = pairsToChange(sortedRoutes(1));
    
    %Takes away the path we want to change from nl
    insertionIndex = find(routeIndices == changeRoute);
    nl(insertionIndex) = [];
    
    %Sets all nodes that are occupied to high cost in order for gsp to not
    %use those nodes.
    piTemp(nl) = reallyHighCost;
    
    %Calculate new path for the pair we are changing path to.
    newRoute = gsp(dimX, dimY, piTemp, 1, com(changeRoute,:) );
    nl = [nl(1:insertionIndex(1)-1); newRoute; nl(insertionIndex(1):end)];
    
    visagrid(dimX,dimY,nl,com,piTemp,shift)
    drawnow
    pause(0.2)

%   Update pi after the new route.
    piTemp = pi;
    piTemp(com) = reallyHighCost; %to prevent paths to be taken over start/end nodes
    
    oldRouteCost = routeCost; %This variable is used to check if we couldn't find any new path when choosing the most expensive one att a collision we choose to change the cheaper one.
    
    % hitta alla kollisioner
    collisionNodes = FindCollisionNodes(nl);
    
    collisionNodes(ismember(collisionNodes, com)) = [];
    
    [routeIndices, routeCost] = UpdateRouteInfo(k,nl,com,piTemp);
    
    %Caclulate cost again
    for i = 1:length(collisionNodes)
        collidingRoutes = routeIndices(find(nl == collisionNodes(i)));
        sameRoutes
        if sameRoutes
            mostExpensiveRoute = sort(routeCost(collidingRoutes));
            disp('cheapest')
        else
            mostExpensiveRoute = sort(routeCost(collidingRoutes),'descend');
            disp('expensive')
        end
        mostExpensiveRoute = find(routeCost == mostExpensiveRoute(1));
        routeCost(mostExpensiveRoute) = routeCost(mostExpensiveRoute) + reallyHighCost;
    end

 %--------------------------------------------------------------
% temporary shit for
% debugging!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    pairsToChange
    %sortedRoutes
    changeRoute

 
    pairsToChange(pairsToChange == changeRoute) = []; % tar bort den rutten som vi nyss ändrat
    
    if isempty(pairsToChange)
        %Create a new vector with pairs we want to change
        pairsToChange = find(routeCost > 3*reallyHighCost);
        if size(nl) == size(oldNl)
            if (oldNl == nl)
                sameRoutes = 1;
            else
                sameRoutes = 0;
            end
        else
            sameRoutes = 0;
        end
        oldNl = nl;
    end
    
    if(iIteration == maxIterations)
        disp(['Iteration limit reached, aborting loop. quitCriteria = ' num2str(maxIterations) ])
        pairsToChange = [];  % causes the while-loop to finish
    end
    iIteration = iIteration + 1;
end

disp('Done!')

%% Take away paths until solution is feasible
   
collisionNodes = FindCollisionNodes(nl);

while ~isempty(collisionNodes)
    
    collidingRoutes = [];
    for i = 1:length(collisionNodes)
        collidingRoutes = [collidingRoutes; routeIndices(find(nl == collisionNodes(i)))];
    end
    
    nCollisions = zeros(1,k);
    for i = 1:k
        nCollisions(i) = length(find(collidingRoutes == i));
    end
    
    mostCollisions = max(nCollisions);
    pairToTakeAway = find(nCollisions == mostCollisions);
    if sum(nCollisions == mostCollisions) > 1
        mostExpensiveRoute = sort(routeCost(collidingRoutes), 'descend');
        mostExpensiveRoute = find(routeCost == mostExpensiveRoute(1));
        pairToTakeAway = mostExpensiveRoute;
    end
    nl(routeIndices == pairToTakeAway) = [];
    com(pairToTakeAway,:) = [];
    k = k - 1;
    
    routeIndices = UpdateRouteInfo(k, nl, com, piTemp);
    
    visagrid(dimX,dimY,nl,com,piTemp,shift)
    drawnow
    pause(0.5)
    
    collisionNodes = FindCollisionNodes(nl);
end
title('Path of the different pairs after the heuristic', 'FontSize', storlek)
disp('Done!')
