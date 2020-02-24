clear all; close all; clc;
rosshutdown();
rosinit();
help = ROSHelper();

help.setHomeSagittal()
disp('home')
pause()

ik = load('./data/ik/ik_piv.mat').ik;
T = length(ik);

disp('place object pls :)')
help.setSimJoints(ik{1}.ik1,ik{1}.ik2);
% help.adddXYZ(1,-1,0,-1,1,0)

pause()

% brings hands down
help.setSimJoints(ik{1}.ik1,ik{1}.ik2);
disp('executing...')
help.setSimJointsTraj(ik)
% for t = 2:T-1
%     help.setSimJoints(ik{t}.ik1,ik{t}.ik2);
%     pause(0.1)
% end

% pause()
% for t = T-1:-1:1
%     help.setSimJoints(ik{t}.ik1,ik{t}.ik2);
%     pause(0.1)
% end