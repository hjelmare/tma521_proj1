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
lambda = 1.99999999;
lambdaFactor = 0.95;
startPi = 1/n;


pi=zeros(n, 1);
for i=1:n
    pi(i)=startPi;
end

% constants we might want to change
maxIteration = 400;
H = zeros(1,maxIteration); %Vector that stores the dualproblem-value at each iteration

for nIteration = 1:maxIteration
   [nl, okcom, x] = LagrangianSubProblem(dimX, dimY, com, k, pi);    %x is the vector that describes the paths, okcom is the paths that were accepted
   
   h = CalculateSubProblem(x, pi, n, okcom); %h is the value of the dual-problem value at this iteration
   
   d = CalcSubGradient(x, n); %d is the subgradient
   
   s = CalcStepLength(lambda, h, d); %s is the step length
   
   pi = UpdatePi(pi, s, d, n);
   
   %updates lambda
   if mod(nIteration, 1) == 0
       lambda = lambda*lambdaFactor;
   end
   
   H(nIteration) = h;
   okNbrOfPaths(nIteration) = length(okcom); %KANSKE TA BORT DENNA, STOG INTE ATT MAN SKULLE HA DEN
                                             %MEN KANSKE VILL HA BILD I
                                             %RAPPORT?
   
end

storlek = 14;
plot(H)
title('Dual-problem value', 'FontSize', storlek)
xlabel('iteration number', 'FontSize', storlek)
ylabel('value', 'FontSize', storlek)
figure
plot(okNbrOfPaths)
title('Accepted paths - dual subproblem', 'FontSize', storlek)
xlabel('iteration number', 'FontSize', storlek)
ylabel('number of accepted paths', 'FontSize', storlek)
disp(['in the end the dualfunction value is ' num2str(H(end)) ' and the number of accepted paths were ' num2str(okNbrOfPaths(end)) ' of ' num2str(k) ' wanted'])

shift = 25;
figure
visagrid(dimX,dimY,nl,com,pi,shift)
title('Path of the different pairs with no heuristic', 'FontSize', storlek)


%% Heuristic

reallyHighCost = 1e6; %Used to modify the paths that the function gsp chooses.

piTemp = pi;
piTemp(com) = reallyHighCost; %to prevent paths to be taken over start/end nodes

% förbered rutternas kostnader med straff för korsning av start och slut
% noder.
[routeIndices, routeCost] = UpdateRouteInfo(k, nl, com, piTemp);

% hitta alla noder där det sker en kollision
collisionNodes = FindCollisionNodes(nl);

%Här hittar vi vilka nod-par som använder de noder som det blir en
%kollision på och väljer att hitta en ny väg för den dyraste av dessa som
%använder samma noder.
collisionNodes(ismember(collisionNodes, com)) = [];
for i = 1:length(collisionNodes)
    collidingPairs = routeIndices(find(nl == collisionNodes(i))); %hittar vilka par som "krockar"
    mostExpensiveIndex = sort(routeCost(collidingPairs), 'descend'); %tar fram den dyraste av de som krockar
    mostExpensiveIndex = find(routeCost == mostExpensiveIndex(1));
    routeCost(mostExpensiveIndex) = routeCost(mostExpensiveIndex) + reallyHighCost;
end

%Förklaring på resonemang: alla par som har en kostnad på mer än 3*vår höga
%kostnad passerar någon "dålig" nod. 2*hög kostnad kommer från varje vägs
%start- och slut-nod
pairsToChange = find(routeCost > 3*reallyHighCost);%Tar fram de par som vi vill byta väg på


iIteration =1;
quitCriteria = 50; %om vi inte har lyckats "trassla ut" alla vägar avbryts den efter 50st. byten
while ~isempty(pairsToChange)
        
    [sortedCosts, sortedRoutes] = sort(routeCost(pairsToChange),'descend'); %Börjar hitta ny väg på den dyraste vägen
    
    % klura ut vilken rutt vi ska ändra på
    collidingRoutes = routeIndices( ismember(nl, collisionNodes) );
    collidingRoutes = unique( collidingRoutes );
    
    changeRoute = pairsToChange(sortedRoutes(1));
    
    % här borde man kika på start/slut-nodsproblemet DETTA SKA VÄL INTE
    % VARA MED?????? KOMENTAREN ALLTSÅ
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
    
    routeIndices = UpdateRouteInfo(k, nl, com, piTemp);
    
    visagrid(dimX,dimY,nl,com,piTemp,shift)
    drawnow
    pause(0.5)
    
    collisionNodes = FindCollisionNodes(nl);
end
title('Path of the different pairs after the heuristic', 'FontSize', storlek)
