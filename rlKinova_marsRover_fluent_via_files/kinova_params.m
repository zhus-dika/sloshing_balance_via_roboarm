% This Script creates variables in the base workspace for
% rlKinovaBallbalance.slx

% Copyright 2021-2022, The MathWorks, Inc.

% --------  время старта RL-агента -------- 
rlStartTime = 0;
rlEpisodeFinalTime = 21;

%% -------- KINOVA ARM PARAMETERS --------

% Initial joint angles
R1_q0 = 0;
R2_q0 = deg2rad(20);
R3_q0 = 0;
R4_q0 = deg2rad(135);
R5_q0 = 0;
R6_q0 = deg2rad(-65);
R7_q0 = deg2rad(-90);

% Initial gripper angle
gAngle0 = 35;

% Max torque limit
% max_torque = [0.0;    0.0;   0.0;    0.0];
% max_torque = [3.5;    0.1;   0.75;    0.5];
% max_torque = [3.0; 0.5; 1.0; 0.3]; agent 6.1-6.7
% max_torque = [2.0; 1.0; 0.75; 0.75];
max_torque = [1.6; 1.2; 0.9; 0.6];   % [J2 J3 J6 J7] Н·м
max_torque_PD = [2.0; 1.0; 0.6; 0.5]; 
robot_opacity = 1;
max_angular_vel = 20;
max_linear_vel = 10;
max_dCOM = 10;
% gravity
g = 3.71;

%% -------- Гравитационная компенсация на старт позы --------------------
% U0 = [11.9 0.1 -1.4 0.09];
% U0 = [11.80, 0.07, -1.24, -0.66]; % log median, N*m
U0 = [11.98, 0.10, -1.22, -0.005]; % log mean
%% -------- GLASS PLATE PARAMETERS --------

plate.length = 0.25;     % m, radius of plate
plate.width  = 0.25;     % m, radius of plate
plate.thickness = 0.005;                    % m, thicknes of plate
plate.mass = 0.2;                        % m, mass of plate

w = plate.width;
h = plate.length;
d = plate.thickness;
plate.moi = 1/12 * plate.mass * [(h^2+d^2), (w^2+d^2), (w^2+h^2)];     % kgm^4, moment of inertia of plate
%% -------- CAD GEOMETRY VISUALIZATION --------
% Needs installation of Robotics System Toolbox Robot Library Data.
% https://www.mathworks.com/matlabcentral/fileexchange/98714-robotics-system-toolbox-robot-library-data

addons = matlab.addons.installedAddons;
spkgInstalled = ismember("RO_Robot_Lib",addons.Identifier);

if spkgInstalled
    spkgroot = matlabshared.supportpkg.getSupportPackageRoot;
    meshPath = fullfile(spkgroot,'toolbox','robotics','supportpackages', ...
        'robotlibrary','Robots','kortex_v12_description','arms', ...
        'gen3','7dof','meshes');
    addpath(meshPath);
end
