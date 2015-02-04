function [ pairs ] = FindPairsUsingNode(nl, node, com)
%Finds (and returns) what pairs that use node "node" in its path.
k = length(com);
for i = 1 : k
    first = last+1;
    slask = find(nl(last+1:length(nl)) == com(i,1));
    last = slask(1)+first-1;
    routeIndices(first:last,1) = i;
end

pairs = routeIndices(find(nl == node));

end

