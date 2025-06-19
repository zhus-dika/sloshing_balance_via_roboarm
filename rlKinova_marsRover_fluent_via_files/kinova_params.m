% This Script creates variables in the base workspace for
% rlKinovaBallbalance.slx

% Copyright 2021-2022, The MathWorks, Inc.

%% -------- VESSEL PARAMETERS --------
vessel.mass = 0.035;
vessel.height = 0.100;

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
max_torque = 0.75;

robot_opacity = 1;

% gravity
g = 9.80665;

% Initial torques
% to find initial torques when ball is at plate center, change the 
% rev joints R6 and R7 to accept motion inputs R6_q0 and R7_q0 and sense 
% the torques.
m2 = 0.206679;
m3 = 0.137977;       % масса локтя
m4 = 0.4;

g = 9.80665;
m_eff_shoulder = m2 + m3 + m4 + vessel.mass;
d_eff_shoulder = 0.2;
shoulder_torque_0 = m_eff_shoulder * g * d_eff_shoulder * cos(R2_q0);

% shoulder_torque_0 = m2 * g * d2 * cos(R2_q0);    

m_eff_elbow = m4 + vessel.mass;   % всё, что висит на локте
d_eff_elbow = 0.12;         % расстояние от локтя до центра масс платформы
elbow_torque_0 = m_eff_elbow * g * d_eff_elbow * cos(R2_q0 + R3_q0);

wrist_torque_0 = (-1.882 + vessel.mass * g) * cos(deg2rad(-65) - R6_q0);
hand_torque_0 = (0.0002349 - vessel.mass * g) * cos(deg2rad(-90) - R7_q0);
U0 = [shoulder_torque_0 elbow_torque_0 wrist_torque_0 hand_torque_0];

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
