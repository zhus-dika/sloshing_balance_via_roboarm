% curr_proj = simulinkproject;

% Load parameters
mars_rover_params_all

open_system("rlKinovaBallBalance")
open_system("rlKinovaBallBalance/Kinova Ball Balance")
kinova_params