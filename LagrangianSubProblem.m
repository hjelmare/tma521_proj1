function [ newnl, okcom, x ] = LagrangianSubProblem(dimX, dimY, com, k, pi )
% Solves the lagrangian function (finds x and solves with respect to pi)
nl = gsp(dimX, dimY, pi, k, com); %finds the ceapest routs

% Calculate cost per route; remove route with
% cost > 1 (required routes stored in nl and pairs in com)

x=zeros(dimX*dimY*2, dimX*dimY*2 , k);
last = 0;
okcom = [];
newnl = [];
for i = 1 : k;
    first = last+1;
    slask = find(nl(last+1:length(nl)) == com(i,1));
    last = slask(1)+first-1;
%    disp(['summa ' num2str(i) '=' num2str(sum( pi( nl(first:last)))) ])
    if (sum(pi(nl(first:last))) < 1)
        okcom = [okcom i];
        newnl = [newnl; nl(first:last)];
        
        %Creates the matrix x_{ijl}
        %--------------------------------
        for j = 0:length(nl(first:last))-2
            from = nl(first+j);
            to = nl(first+j+1);
            x(from, to, i) = 1;
        end
        %--------------------------------
        
    end
end

%create x_{ijl}
    

end

