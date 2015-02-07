%% Heuristic

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
routesToChange = find(routeCost > 3*reallyHighCost);
oldNl = nl; %used to check if something has changed in the while-loop.

sameRoutes = 0;
iIteration =1;
while ~isempty(routesToChange)   % Stop when there are no more collisions
    % Start looking for the worst route (most collisions, highest cost)
    [sortedCosts, sortedRoutes] = sort(routeCost(routesToChange),'descend');
    changeRoute = routesToChange(sortedRoutes(1));
    
    % Take away the path we want to change from nl
    insertionIndex = find(routeIndices == changeRoute);
    nl(insertionIndex) = [];
    
    % Sets all nodes that are occupied to high cost in order for gsp to not
    % use those nodes.
    piTemp(nl) = reallyHighCost;
    
    % Calculate new path for the route we are changing
    newRoute = gsp(dimX, dimY, piTemp, 1, com(changeRoute,:) );
    % and insert it in the node list
    nl = [nl(1:insertionIndex(1)-1); newRoute; nl(insertionIndex(1):end)];
    
    visagrid(dimX,dimY,nl,com,piTemp,shift)
    drawnow
    pause(viewTime)

%   Update pi after the new route.
    piTemp = pi;
    piTemp(com) = reallyHighCost; %to prevent paths to be taken over start/end nodes
    
    oldRouteCost = routeCost; %This variable is used to check if we couldn't find any new path when choosing the most expensive one att a collision we choose to change the cheaper one.
    
    % Find nodes with collisions on them
    collisionNodes = FindCollisionNodes(nl);
    % but remove the start/end nodes from the list
    collisionNodes(ismember(collisionNodes, com)) = [];
    % because they already have penalties on them
    
    [routeIndices, routeCost] = UpdateRouteInfo(k,nl,com,piTemp);
    
    %Caclulate cost again
    for i = 1:length(collisionNodes)
        collidingRoutes = routeIndices( (nl == collisionNodes(i)) );
        
        if sameRoutes
            mostExpensiveRoute = sort(routeCost(collidingRoutes));
        else
            mostExpensiveRoute = sort(routeCost(collidingRoutes),'descend');
        end
        mostExpensiveRoute = find(routeCost == mostExpensiveRoute(1));
        routeCost(mostExpensiveRoute) = routeCost(mostExpensiveRoute) + reallyHighCost;
    end

    % Remove the route we just changed from the list of routes to change
    routesToChange(routesToChange == changeRoute) = [];
    
    if isempty(routesToChange)
        %Create a new vector with pairs we want to change
        routesToChange = find(routeCost > 3*reallyHighCost);
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
        disp(['Iteration limit reached, aborting loop. maxIterations = ' num2str(maxIterations) ])
        routesToChange = [];  % causes the while-loop to finish
    end
    iIteration = iIteration + 1;
end

disp('Untangling done!')

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
    pause(viewTime)
    
    collisionNodes = FindCollisionNodes(nl);
end

% Change title of plot
title('Path of the different pairs after the heuristic', 'FontSize', storlek)

disp('Feasibility done!')
