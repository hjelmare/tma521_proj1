%% Given dimensions and contact paris, this program calculates the maximum
% numbers of pairs that can be made.
clear all
clc
addpath(pwd);

%initialization
%--------Example from dpf (to test gsp)--------
disp('OBS! VI FÅR INTE SAMMA RESULTAT SOM HON FÅR I SIN PDF') 
k=3;
dimX=8;
dimY=6;
com = [1 48; 3 41; 5 42];

%-------------------

n = dimX*dimY*2;    %number of nodes
lambda = 1.99999999;
lambdaFactor = 0.95;
startPi = 0.1; %CHANGE THIS TO 1/n LATER!!!!!!!!!!!!


pi=zeros(n, 1);
for i=1:n
    pi(i)=startPi;
end

% constants we might want to change
maxIteration = 1;
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
end

