clear all; close all; clc;
rosshutdown();
rosinit();
help = ROSHelper();

help.setHomeSagittal()

disp('home')
pause()

p = load('./data/mats/sq_grasp.mat').traj*700;
N_c = size(p,2);
T = size(p,4);

dp = zeros(2,N_c,1,T);
for t = 1:T-1
    for c = 1:N_c
        dp(:,c,1,t) = p(:,c,1,t+1) - p(:,c,1,t);
    end
end

if size(p,2) < 2
    help.setInitialPositionSagittal(p(1,1,1,1),p(2,1,1,1),-200,p(2,1,1,1))
else 
    help.setInitialPositionSagittal(p(1,1,1,1),p(2,1,1,1),p(1,2,1,1),p(2,2,1,1))
end

pause()

% brings hands down
help.adddXYZ(-1,-4,-54,1,4,-54)
% for c = 1:N_c
%     help.setdXYZ(c,0,0,-54)
% end

disp('place object pls :)')
pause()
help.adddXYZ(0,10,0,0,-10,0)
pause(0.1)

for t = 1:T-1
    for c = N_c:-1:1
        help.setdXYZ(c,0,dp(1,c,1,t),dp(2,c,1,t));
        pause(0.1)
    end
    % pause(0.1)
end

pause()

for t = T-1:-1:1
    for c = 1:N_c
        help.setdXYZ(c,0,-dp(1,c,1,t),-dp(2,c,1,t));
        pause(0.1)
    end
    % pause(0.1)
end