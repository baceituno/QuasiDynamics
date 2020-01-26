clear all; close all; clc;

% defines thez object
% task_1 = square_pivot();
% task_1 = square_sliding();
task_1 = triangle_pivot();

object = {task_1};
sols = {};

% presolves for contact schedule
for i = 1:length(object)
	pre = MixedIntegerContactPlacementProblem(object{i},1,2)
	pre.McCormick = 1;

	pre = pre.addDynamicsConstraints();
	pre = pre.addContactConstraints();
	pre = pre.addNonPenetrationConstraints();
	pre = pre.addCostFunction();

	pre = pre.solve();
	sols{i} = pre;
end

% defines the program
prog = MixedIntegerEffectorDesignProblem(object,1,2,5);
prog.McCormick = 1;
prog.McCormick2 = 1;
prog.M = 6;

prog = prog.addDynamicsConstraints();
prog = prog.addContactConstraints(sols);
prog = prog.addRigidShapeConstraints();
prog = prog.addAssignmentConstraints(sols);
prog = prog.addNonPenetrationConstraints();
prog = prog.addCostFunction();

prog = prog.solve()

% refines the shape


% Animaation (IMRPOVE COLORS/ARROWS)
animation_mip(object{1},prog,prog)