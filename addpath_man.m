% adds all of the manipulation code to the path and gurobi (for MAC OS)
display('adding code to path');
current_fldr = pwd;
cd /Library/gurobi810/mac64/examples/matlab
cd ../..
cd matlab
gurobi_setup
cd (current_fldr)
root = fileparts(mfilename('fullpath'));
addpath(genpath(root))