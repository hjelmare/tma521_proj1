function [ routeIndices , routeCost] = UpdateRouteInfo( k, nl, com, cost)
%UPDATEROUTEINDICES Summary of this function goes here
%   Detailed explanation goes here

last = 0;
for i = 1 : k;
    first = last+1;
    slask = find(nl(last+1:length(nl)) == com(i,1));
    last = slask(1)+first-1;
    routeCost(i) =  sum(cost(nl(first:last)));
    routeIndices(first:last,1) = i;
end



end

