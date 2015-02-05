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
maxIteration = 1200;
for nIteration = 1:maxIteration
   [nl, okcom, x] = LagrangianSubProblem(dimX, dimY, com, k, pi);    %x is the vector that describes the paths, okcom is the paths that were accepted
   
   h = CalculateSubProblem(x, pi, n, okcom);
   
   d = CalcSubGradient(x, n); %CHECK THIS! THIS MIGHT BE WRONG
   
   s = CalcStepLength(lambda, h, d);
   
   pi = UpdatePi(pi, s, d, n);
   
   %updates lambda
   if mod(nIteration, 10) == 0
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

%% test constraint 1c:
%Detta gör inte sitt jobb rätt
for i =1:n
    for j=1:k
        totalt(i) = sum(x(:,i,j));
    end
end


%% Heuristic

reallyHighCost = 1e6;

piTemp = pi;
piTemp(com) = reallyHighCost; %to prevent paths to be taken over start/end nodes

% hitta alla kollisioner
collisionNodes = [];
for i = 1:length(nl)
    if length(find(nl == nl(i))) > 1
        if ~ismember(nl(i),collisionNodes)
            collisionNodes = [collisionNodes ; nl(i)];
        end
    end
end
    
% förbered rutternas kostnader och så...
last = 0;
for i = 1 : k;
    first = last+1;
    slask = find(nl(last+1:length(nl)) == com(i,1));
    last = slask(1)+first-1;
    routeCost(i) =  sum(pi(nl(first:last))) + reallyHighCost * sum(ismember(nl(first:last),com));
    routeIndices(first:last,1) = i;
end



[sortedCosts, sortedRoutes] = sort(routeCost,'descend');

% klura ut vilken rutt vi ska ändra på
collidingRoutes = routeIndices( ismember(nl, collisionNodes) );
collidingRoutes = unique( collidingRoutes );

i = 1;
while ~ismember(sortedRoutes(i),collidingRoutes)
    i = i + 1;
end
changeRoute = sortedRoutes(i);

% här borde man kika på start/slut-nodsproblemet

nl(find(routeIndices == changeRoute)) = [];

piTemp(nl) = reallyHighCost;

newRoute = gsp(dimX, dimY, piTemp, 1, com(changeRoute,:) );

%nl = [nl ; newRoute];

% klipp in manuellt

%%

shift = 25;
figure
visagrid(dimX,dimY,nl,com,piTemp,shift)



% startEndConflict = com(ismember(com, collisionNodes));
% 
% 
% while(length(startEndConflict) > 0)
%     conflictingPairs = FindPairsUsingNode(nl, startEndConflict(1), com);
%     startEndConflict(1) = [];
%     
%     while(length(conflictPairs) > 1 )
%         
%     end
% end

% här ska man skriva saker :)

% ta bort den ur nl
% sätt alla noder som är kvar i nl till dyra
% hitta en ny rutt enligt com[borttagen rutt]
% infoga den och börja om.



