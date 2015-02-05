%% Given dimensions and contact paris, this program calculates the maximum
% numbers of pairs that can be made.
clear all
clc
close all
addpath(pwd);

%initialization
%-------p6.m----------------------------------
dimX=8;
dimY=6;
k=7;
com = [1 45; 
       2 43; 
       3 44; 
       4 42; 
       5 46; 
       6 47; 
       7 48];
%--------------------------------------------

%------p10.m-------------------------------
% dimX=30;
% dimY=10;
% k=15;
% com = [24   271; 
%         8   282; 
%        21   295; 
%        25   290; 
%        15   291; 
%        27   279; 
%        30   293; 
%        26   289;  
%         2   297;
%        29   284; 
%        22   276; 
%        16   288; 
%        19   273; 
%        10   275;  
%         7   286];
%----------------------------------------

%-----------p11.m-----------------------
% dimX=30;
% dimY=11;
% k=15;
% 
% com = [24   301; 
%         8   312; 
%        21   325; 
%        25   320; 
%        15   321; 
%        27   309; 
%        30   323; 
%        26   319;  
%         2   327;
%        29   314; 
%        22   306; 
%        16   318; 
%        19   303; 
%        10   305;  
%         7   316];
%-----------------------------

n = dimX*dimY*2;    %number of nodes
H = zeros(1,n);
lambda = 1.99999999;
lambdaFactor = 0.95;
startPi = 1/n;


pi=zeros(n, 1);
for i=1:n
    pi(i)=startPi;
end

% constants we might want to change
maxIteration = 400;
for nIteration = 1:maxIteration
   [nl, okcom, x] = LagrangianSubProblem(dimX, dimY, com, k, pi);    %x is the vector that describes the paths, okcom is the paths that were accepted
   
   h = CalculateSubProblem(x, pi, n, okcom);
   
   d = CalcSubGradient(x, n); %CHECK THIS! THIS MIGHT BE WRONG
   
   s = CalcStepLength(lambda, h, d);
   
   pi = UpdatePi(pi, s, d, n);
   
   %updates lambda
   if mod(nIteration, 1) == 0
       lambda = lambda*lambdaFactor;
   end
   
   H(nIteration) = h;
   okNbrOfPaths(nIteration) = length(okcom);
   
end

plot(H)
figure
plot(okNbrOfPaths)
disp(['in the end the dualfunction value is ' num2str(H(end)) ' and the number of accepted paths were ' num2str(okNbrOfPaths(end)) ' of ' num2str(k) ' wanted'])

%% Testing the program. Plotting and such
shift = 25;
figure
visagrid(dimX,dimY,nl,com,pi,shift)


%% Heuristic

reallyHighCost = 1e6;

piTemp = pi;
piTemp(com) = reallyHighCost; %to prevent paths to be taken over start/end nodes

% förbered rutternas kostnader med straff för korsning av start och slut
% noder.
routeCost = zeros(k, 1);
shift = 25;
last = 0;
for i = 1 : k;
    first = last+1;
    slask = find(nl(last+1:length(nl)) == com(i,1));
    last = slask(1)+first-1;
    routeCost(i) =  sum(piTemp(nl(first:last)));
    routeIndices(first:last,1) = i;
end

% hitta alla kollisioner
collisionNodes = FindCollisionNodes(nl);

collisionNodes(ismember(collisionNodes, com)) = [];
for i = 1:length(collisionNodes)
    collidingPairs = routeIndices(find(nl == collisionNodes(i)));
    mostExpensiveIndex = sort(routeCost(collidingPairs), 'descend');
    mostExpensiveIndex = find(routeCost == mostExpensiveIndex(1));
    routeCost(mostExpensiveIndex) = routeCost(mostExpensiveIndex) + reallyHighCost;
end


pairsToChange = find(routeCost > 3*reallyHighCost);

iIteration =1;
quitCriteria = 50;
while ~isempty(pairsToChange)
        
    [sortedCosts, sortedRoutes] = sort(routeCost(pairsToChange),'descend');
    routeCost
    pairsToChange
    sortedRoutes
    
    % klura ut vilken rutt vi ska ändra på
    collidingRoutes = routeIndices( ismember(nl, collisionNodes) );
    collidingRoutes = unique( collidingRoutes );
    
    changeRoute = pairsToChange(sortedRoutes(1));
    
    % här borde man kika på start/slut-nodsproblemet
    insertionIndex = find(routeIndices == changeRoute);
    nl(insertionIndex) = [];
    
    piTemp(nl) = reallyHighCost;
    
    newRoute = gsp(dimX, dimY, piTemp, 1, com(changeRoute,:) );
    nl = [nl(1:insertionIndex(1)-1); newRoute; nl(insertionIndex(1):end)];
    
    visagrid(dimX,dimY,nl,com,piTemp,shift)
    drawnow
    pause(0.2)

%----------------------Calculating cost-----------------------------
    % beräknar rutternas kostnader och så...    
    piTemp = pi;
    piTemp(com) = reallyHighCost; %to prevent paths to be taken over start/end nodes
    last = 0;
    routeIndices=zeros(length(nl),1);
    oldRouteCost = routeCost;
    for i = 1 : k;
        first = last+1;
        slask = find(nl(last+1:length(nl)) == com(i,1));
        last = slask(1)+first-1;
        routeCost(i) =  sum(piTemp(nl(first:last)));
        routeIndices(first:last,1) = i;
    end
    
    % hitta alla kollisioner
    collisionNodes = FindCollisionNodes(nl);
    
    collisionNodes(ismember(collisionNodes, com)) = [];
    for i = 1:length(collisionNodes)
        collidingPairs = routeIndices(find(nl == collisionNodes(i)));
        if oldRouteCost == routeCost
            mostExpensiveIndex = sort(routeCost(collidingPairs));
        else
            mostExpensiveIndex = sort(routeCost(collidingPairs),'descend');
        end
        mostExpensiveIndex = find(routeCost == mostExpensiveIndex(1));
        routeCost(mostExpensiveIndex) = routeCost(mostExpensiveIndex) + reallyHighCost;
    end
 %--------------------------------------------------------------
    
    pairsToChange = pairsToChange(2:end); % tar bort den rutten som vi nyss ändrat
    
    if isempty(pairsToChange)
       pairsToChange = find(routeCost > 3*reallyHighCost);
    end
    
    if(iIteration == quitCriteria)
        disp(['den hann inte reda ut allt, vi avbröt while-loopen. \n quitCriteria = ' num2str(quitCriteria) ])
        pairsToChange = [];
    end
    iIteration = iIteration + 1;
end
%% EJ KLAR - STÄMMER LOOPEN?
   
collisionNodes = FindCollisionNodes(nl);

while ~isempty(collisionNodes) %Takes away pairs that can't make a feasible path.
    
    collisionNodes = FindCollisionNodes(nl);
    
    for i = 1:length(collisionNodes)
        collidingPairs = [collidingPairs; routeIndices(find(nl == collisionNodes(i)))];
    end
    
    nbrOfCollisions = zeros(1,k);
    for i = 1:k
        nbrOfCollisions(i) = length(find(collidingPairs == i));
    end
    
    if sum(nbrOfCollisions == max(nbrOfCollisions)) > 1
        [~, pairToTakeAway] = max(nbrOfCollisions);
        mostExpensiveIndex = sort(routeCost(collidingPairs), 'descend');
        mostExpensiveIndex = find(routeCost == mostExpensiveIndex(1));
        nl(routeIndices == pairToTakeAway) = [];
        com(pairToTakeAway,:) = [];
        
    else
        [~, pairToTakeAway] = max(nbrOfCollisions);
        nl(routeIndices == pairToTakeAway) = [];
        com(pairToTakeAway,:) = [];
    end
    k = k - 1;
    
    visagrid(dimX,dimY,nl,com,piTemp,shift)
    drawnow
    pause(1.5)

    collisionNodes = FindCollisionNodes(nl);
end

