clear all
clc
close all
addpath(pwd);

% Select which problem you want to try and solve
  p6
% p10
% p11

n = dimX*dimY*2;    % Number of nodes

% Some settings that one might want to change
maxIterations = 1000;
lambda = 1.99999999;
lambdaIterations = 15;
startPi = 1/n;          % Initial value for pi (the cost)

SolveTask1