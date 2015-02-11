function [ routeIndices , routeCost] = UpdateRouteInfo( k, nl, com, cost)
% Calculates the cost of each route, and makes a list corresponding to
% the node list nl containing the contact pair number that each entry in
% nl belongs to.

last = 0;
for i = 1 : k;
    first = last+1;
    slask = find(nl(last+1:length(nl)) == com(i,1));
    if ~isempty(slask)
        last = slask(1)+first-1;
        
        routeCost(i) =  sum(cost(nl(first:last)));
        routeIndices(first:last,1) = i;
    else
        routeCost(i) = 0;
    end
end

end

