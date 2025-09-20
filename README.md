## 🐠 Software

- Matlab R2024a
- Ansys 2025R1

## 🦋 Used models from the Mathworks Team

- Train Kinova RoboArm https://www.mathworks.com/help/reinforcement-learning/ug/train-sac-agent-for-ball-balance-control.html
- Mars Rover https://www.mathworks.com/help/sm/ug/mars_rover.html
## 🪲 GUIDE
### 🐱 SIMULINK
- Open project file `rlKinovaBallBalance.prj`
- For 3d visualisation, change `Visualisation` option from `None` to `3D Mesh`
- Run `TrainSACAgentForVesselWithFluidControlExample.mlx` script


  **Main physical scheme**
![alt text](https://github.com/zhus-dika/sloshing_balance_via_roboarm/blob/main/rlKinova_marsRover_fluent_via_files/images/main_scheme.png)

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
- Read case file [glass_liquid_sloshing_simulink/vessel-res-2/vessel_3.5x6.0x100_res-2.cas.h5](glass_liquid_sloshing_simulink/vessel-res-2/vessel_3.5x6.0x100_res-2.cas.h5)
- Set the number of MPI processes, for example, for 4 cores is 4
- Build & load udf file [glass_liquid_sloshing_simulink/vessel-res-2/io_fluent-simulink_udf_vessel-res-2.c](glass_liquid_sloshing_simulink/vessel-res-2/io_fluent-simulink_udf_vessel-res-2.c)
- Activate function hooks
- Read a scheme via GUI *File → Read → Scheme* [glass_liquid_sloshing_simulink/vessel-res-2/reset/reset_flag_vessel-res-2.scm](glass_liquid_sloshing_simulink/vessel-res-2/reset/reset_flag_vessel-res-2.scm)
- Add cmd ```(reset-if-flag)``` in Execute Commands via GUI *Solution → Calculation Activities → Execute Commands*
  Details: Active — ✔︎, Execution Type → Execute Repeatedly, Every = 1

### 🦏 Animation 

- Save video in Mechanics Explorer as MJPEG-AVI in Matlab
    ```
    mdl = 'rlKinovaSloshingBalance';
    
    smwritevideo(mdl,'rover_100fps.avi', ...
     'VideoFormat','motion jpeg avi', ...
     'FrameRate',100, ...
     'FrameSize',[1920 1080], ...
     'PlaybackSpeedRatio',1.0);
    ```
- Convert avi to mp4 in bash
  ```
  ffmpeg -i data/output/rover_100fps.avi \
    -c:v libx264 -pix_fmt yuv420p -crf 18 -preset medium -r 100 -movflags +faststart \
    data/output/rover_100fps.mp4
  ```
![Alt Text](https://github.com/zhus-dika/sloshing_balance_via_roboarm/blob/main/rlKinova_marsRover_fluent_via_files/data/output/rover-side_slosh_pip_time_100fps.gif)

### 🐠 Useful bash cmds
Allocate additional swap

```sudo fallocate -l 4G /swapfile2```

```sudo chmod 600 /swapfile2```

```sudo mkswap /swapfile2```

```sudo swapon /swapfile2```
