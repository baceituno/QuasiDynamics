clear all; close all; clc;

% initializes the ROSHelper
rosshutdown;
rosinit;
helper = ROSHelper();

helper.calibrateHand(1);
helper.calibrateHand(2);

% finds a task
task = square_pivot();

% infers the primitive
pre = MixedIntegerContactPlacementProblem(task,1,2)
pre.McCormick = 1;
pre.M = 12;

% animation_shape(task,false)
% pause()

pre = pre.addDynamicsConstraints();
pre = pre.addContactConstraints();
pre = pre.addNonPenetrationConstraints();
pre = pre.addCostFunction();

pre = pre.solve();

% animates and saves the video
animation_contacts(task,pre,false)

%% Sending initial commands to robot
helper.setHomePlanarSagittal(pi/8);

% Move
helper.setSimDeltaXYZ(0,0,-20);
 
% Sends each segment of the plan 
display('Performing initial approach...')
helper.clearBuffers();
for t = 1:T+1
    
    p1 = 2000*pre.vars.p.value(:,1,t);
    p2 = 2000*pre.vars.p.value(:,2,t);
    if t ~= T
        helper.addHandsToBuffer(p1, p2);
    end

    if t == 1
		helper.executeSimBuffers(true);
        display('Initial approach performed. Press any key to execute plan.')
        % pause()
        % helper.setSimDeltaXYZ(0,0,-18);
        helper.clearBuffers();
        % pause()
    end
end

helper.executeSimBuffers(true);
display('Plan executed. Press any key to go home.')
pause()

helper.clearBuffers();
helper.setSimJointsTraj()