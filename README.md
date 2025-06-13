## 🦋 Used models from the Mathworks Team

- Train Kinova RoboArm https://www.mathworks.com/help/reinforcement-learning/ug/train-sac-agent-for-ball-balance-control.html
- Mars Rover https://www.mathworks.com/help/sm/ug/mars_rover.html
## 🪲 GUIDE
### 🐱 SIMULINK
- Open project file `rlKinovaBallBalance.prj`
- For 3d visualisation, change `Visualisation` option from `None` to `3D Mesh`
- Run `TrainSACAgentForBallBalanceControlExample.mlx` script
### 🐌 FLUENT
- Run ansys server in bash
    ```
    ~/Documents/ansys_inc/shared_files/licensing/start_lmcenter
    ```
- Start license manager & open http://localhost:1084/ in web browser & put button **start**
- Run Fluent
    ```
    __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia fluent
    ```
- Read case file [glass_liquid_sloshing_simulink/FFF_last.cas.h5](glass_liquid_sloshing_simulink/FFF_last.cas.h5)
- Set the number of MPI processes, for example, for 4 cores is 4
- Load udf file [glass_liquid_sloshing_simulink/io_fluent-simulink_udf_final.c](glass_liquid_sloshing_simulink/io_fluent-simulink_udf_final.c)
- Activate function hooks