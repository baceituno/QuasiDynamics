clear all; close all; clc;
rosshutdown();
rosinit();
help = ROSHelper();

help.setHomePlanar()

disp('home')
pause()

p = load('./data/mats/p_rot.mat').traj*500;
N_c = size(p,2);
T = size(p,4);

dp = zeros(2,N_c,1,T);
for t = 1:T-1
    for c = 1:N_c
        dp(:,c,1,t) = p(:,c,1,t+1) - p(:,c,1,t);
    end
end

if size(p,2) < 2
    help.setInitialPositionPlanar(p(1,1,1,1),p(2,1,1,1),-200,100)
else 
    help.setInitialPositionPlanar(p(1,1,1,1),p(2,1,1,1),p(1,2,1,1),p(2,2,1,1))
end

pause()

% brings hands down
for c = 1:N_c
    help.setdXYZ(c,0,0,-50)
end

disp('place object pls :)')
pause()

for t = 1:T-1
    for c = 1:N_c
        help.setdXYZ(c,dp(1,c,1,t),dp(2,c,1,t),0);
        pause(0.1)
    end
    % pause()
end