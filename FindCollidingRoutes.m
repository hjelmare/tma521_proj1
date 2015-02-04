%% används inte just nu

function [ collidingRoutes, routeCost, collidingNode ] = FindCollidingRoutes( nl, pi, com )
%FINDCOLLIDINGROUTES Summary of this function goes here
%   Detailed explanation goes here

k = length(com);
n = length(pi);

routeIndices = zeros(length(nl),1);
last = 0;
for i = 1 : k;
    first = last+1;
    slask = find(nl(last+1:length(nl)) == com(i,1));
    last = slask(1)+first-1;
    routeCost(i) =  sum(pi(nl(first:last)));
    routeIndices(first:last,1) = i;
end

i = 1;
node = nl(i);
while ( length(find(nl == node)) <= 1 ) && ( i <= n)
    i = i + 1;
    node = nl(i);
end

collidingIndices = find(nl == nl(i));

collidingRoutes = routeIndices(collidingIndices);
routeCost = routeCost(collidingRoutes);
collidingNode = nl(i);

end

