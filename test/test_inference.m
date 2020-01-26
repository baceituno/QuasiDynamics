clear all; close all; clc;

% finds a task
task = square_line();

% infers the primitive
pre = MixedIntegerContactPlacementProblem(task,1,2)
pre.McCormick = 1;
pre.M = 4;

% animation_shape(task,false)
% pause()

pre = pre.addDynamicsConstraints();
pre = pre.addContactConstraints();
pre = pre.addNonPenetrationConstraints();
pre = pre.addCostFunction();

c = tic()
pre = pre.solve();
toc(c)

% animates and saves the video
animation_contacts(task,pre,false)

% SCS solver for MIQP