function [ collisionNodes ] = FindCollisionNodes( nl )
% Lists all nodes that occur more than once in the node list.
% The list contains only unique entries

collisionNodes = [];
    for i = 1:length(nl)
        if length(find(nl == nl(i))) > 1
            if ~ismember(nl(i),collisionNodes)
                collisionNodes = [collisionNodes ; nl(i)];
            end
        end
    end



end

