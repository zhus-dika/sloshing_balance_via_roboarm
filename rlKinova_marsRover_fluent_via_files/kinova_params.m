% This Script creates variables in the base workspace for
% rlKinovaBallbalance.slx

% Copyright 2021-2022, The MathWorks, Inc.

%% -------- VESSEL PARAMETERS --------
vessel.mass = 0.035;
vessel.mass_liquid = 4.821742e-01;
vessel.height = 0.100;
vessel.radius_top = 0.06;
vessel.radius_bot = 0.035;
vessel.fill = 0.077;

fluent0.m = 4.821742e-01;
fluent0.com = [-4.162623e-03 -4.364569e-02 6.975493e-02];
fluent0.I6 = [2.656169e-03 2.656171e-03 2.828628e-03 -4.701965e-09 4.807127e-08 -1.838260e-08];

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
max_torque = [3.5;    0.1;   0.75;    0.5];
% max_torque = [1.5;    0.05;   0.5;    0.05];

robot_opacity = 1;

% gravity
g = 3.71;

%% -------- Гравитационная компенсация на старт позы --------------------
U0 = [11.9 0.1 -1.4 0.09];

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
