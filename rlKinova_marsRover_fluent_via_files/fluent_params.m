%% -------- VESSEL PARAMETERS --------
vessel.mass = 0.035;
vessel.mass_liquid = 4.35e-01;
vessel.height = 0.100;
vessel.radius_top = 0.06;
vessel.radius_bot = 0.035;
vessel.fill = 0.077;

fluent0.m = vessel.mass + vessel.mass_liquid;
fluent0.com = [0 0 3.82e-02];
fluent0.I6 = [2.656169e-03 2.656171e-03 2.828628e-03 -4.701965e-09 4.807127e-08 -1.838260e-08];
feed0 = [ 0;                       % 1: time (ниже по схеме не используется)
          fluent0.m;               % 2: m_tot
          vessel.mass_liquid; % 3: m_liq
          fluent0.com(:);          % 4:6: COM в кадре сосуда
          fluent0.I6(:);           % 7:12: [Ixx Iyy Izz Ixy Iyz Ixz]
          0 ];                     % 13: spill за шаг