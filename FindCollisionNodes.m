function [ collisionNodes ] = FindCollisionNodes( nl )
%FINDCOLLISIONNODES Summary of this function goes here
%   Detailed explanation goes here

collisionNodes = [];
    for i = 1:length(nl)
        if length(find(nl == nl(i))) > 1
            if ~ismember(nl(i),collisionNodes)
                collisionNodes = [collisionNodes ; nl(i)];
            end
        end
    end



end

