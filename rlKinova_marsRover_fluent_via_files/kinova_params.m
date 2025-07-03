% This Script creates variables in the base workspace for
% rlKinovaBallbalance.slx

% Copyright 2021-2022, The MathWorks, Inc.

%% -------- VESSEL PARAMETERS --------
vessel.mass = 0.035;
vessel.mass_liquid = 0.5;
vessel.height = 0.100;
vessel.radius_top = 0.06;
vessel.radius_bot = 0.035;
vessel.fill = 0.077;

fluent0.m = 5.272592e-01;
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
max_torque = 0.75;

robot_opacity = 1;

% gravity
g = 9.80665;

% Initial torques
% to find initial torques when ball is at plate center, change the 
% rev joints R6 and R7 to accept motion inputs R6_q0 and R7_q0 and sense 
% the torques.
%% -------- Гравитационная компенсация на старт позы --------------------
% массы звеньев Gen3 (кг)
m2 = 0.206679;
m3 = 0.137977;     % локоть
m4 = 0.400000;     % «предпредплечье» (R6-звено)

% суммарная масса груза на плечевом и локтевом моторах
g  = 9.80665;                            % м/с²
mCup      = vessel.mass + vessel.mass_liquid;

m_eff_sh  = m2 + m3 + m4 + mCup;         % всё, что «висит» на R2
d_sh      = 0.20;                        % плечо до суммарного COM
shoulder_torque_0 = m_eff_sh * g * d_sh * sin(R2_q0);   % Н·м

m_eff_el  = m3 + m4 + mCup;              % всё, что «висит» на R3
d_el      = 0.12;                        % плечо до COM груза
elbow_torque_0    = m_eff_el * g * d_el * sin(R2_q0 + R3_q0);

% для запястья и ладони, если нет точных плеч ‒ оставим 0
wrist_torque_0 = 0;
hand_torque_0  = 0;

U0 = [shoulder_torque_0 elbow_torque_0 wrist_torque_0 hand_torque_0];

%% -------- GLASS PLATE PARAMETERS --------

plate.length = 0.25;     % m, radius of plate
plate.width  = 0.25;     % m, radius of plate
plate.thickness = 0.005;                    % m, thicknes of plate
plate.mass = 0.2;                        % m, mass of plate

plate.linear_vel_limit = 4;
plate.angle_vel_limit = 8;

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
