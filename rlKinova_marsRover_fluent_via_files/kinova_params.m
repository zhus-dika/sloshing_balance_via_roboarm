% This Script creates variables in the base workspace for
% rlKinovaBallbalance.slx

% Copyright 2021-2022, The MathWorks, Inc.
% --------  время старта fluent -------- 
fluentStartTime = 10;

% --------  время старта RL-агента -------- 
rlStartTime = 0;
rlTimeStep = 0.01;
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
max_torque = [1.6; 0.6; 0.9; 0.5];   % [J2 J3 J6 J7] Н·м
delta_tau = [0.05*1.6; 0.05*0.6; 0.10*0.9; 0.10*0.5];  % = [0.08; 0.03; 0.09; 0.05] Н·м/шаг
max_torque_PD = [2.5; 0.6; 0.6; 0.3]; 
robot_opacity = 1;
max_angular_vel = 2.0;
max_linear_vel = 0.6;
max_dCOM = 10;
% gravity
g = 3.71;

%% -------- Гравитационная компенсация на старт позы --------------------
U0 = [-5.75, 0.10, -1.3, 0.0]; % log mean
abs_max_torque_U = [6, 0.70, 2.12, 0.505];
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
%% ---------- g(q): математическая копия руки ----------
% robot = loadrobot("kinovaGen3", DataFormat="column", Gravity=[0 0 -g]);
% q0_full = [R1_q0; R2_q0; R3_q0; R4_q0; R5_q0; R6_q0; R7_q0];  % радианы
% showdetails(robot)
