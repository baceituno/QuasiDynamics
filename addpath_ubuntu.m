% adds all of the manipulation code with Drake and Gurobi to the path for ubuntu
display('adding code to path');
% addpath_drake
current_fldr = pwd;
cd /opt/gurobi900/linux64/matlab
gurobi_setup
cd (current_fldr)
root = fileparts(mfilename('fullpath'));
addpath(genpath(root))