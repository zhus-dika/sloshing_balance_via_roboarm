## 🦎 Motivation
This project focuses on active sloshing control for planetary rover operations. A partially filled vessel is mounted on a rover and equipped with a Kinova Gen3 robotic arm that serves as an active damper for the moving liquid. Using Simscape Multibody–Fluent co-simulation, we model the interaction between the rover dynamics, the arm motion, and the free-surface liquid under reduced-gravity conditions (e.g., Mars). A reinforcement learning controller (SAC) is trained to generate arm motions that attenuate sloshing, limit spills, and stabilize the liquid center of mass while the rover follows randomized paths.
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
### 🐌 FLUENT
- Run ansys server in bash
    ```
    ~/Documents/ansys_inc/shared_files/licensing/start_lmcenter
    ```
- Start license manager & open http://localhost:1084/ in web browser & put button START
- Run Fluent
    ```
    __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia fluent
    ```
- Read case&data file [glass_liquid_sloshing_simulink/vessel-res-2/vessel_3.5x6.0x100_res-2.cas.h5](glass_liquid_sloshing_simulink/vessel-res-2/vessel_3.5x6.0x100_res-2.cas.h5)
- Set the number of MPI processes, for example, for 4 cores is 4
- Build & load udf file [glass_liquid_sloshing_simulink/vessel-res-2/io_fluent-simulink_udf_vessel-res-2.c](glass_liquid_sloshing_simulink/vessel-res-2/io_fluent-simulink_udf_vessel-res-2.c)
- Activate function hooks
- Read a scheme via GUI *File → Read → Scheme* [glass_liquid_sloshing_simulink/vessel-res-2/reset/reset_flag_vessel-res-2.scm](glass_liquid_sloshing_simulink/vessel-res-2/reset/reset_flag_vessel-res-2.scm)
- Add cmd ```(reset-if-flag)``` in Execute Commands via GUI

  *Solution → Calculation Activities → Execute Commands*

  Details: Active — ✔︎, Execution Type → Execute Repeatedly, Every = 1
  
## 🦤 About observation & reward
### 🦋 Observations are:
 1. positions (sine & cosine of joint angles) & velocities (joint angle derivatives) of the 4 actuated joints {1-12}
 2. volume of spills related to m_0 {13}
 3. COM of the liquid from FLUENT {14-16}
 4. dCOM velocity {17-19}
 5. normal vector of the GVM block {20-22} 
 6. lambda parameter (m_liquid/m_0) {23}
 7. angular & linear velocities (related to platform) of the GVM block {24-29}
 8. rover's acceleration related to GVM local system a_GVM_loc=quatToR_W2vessel{qPlate}(Mars_gravity-a_GVM) {30-32}
### 🐡 Rewards are:
 1. Penalty for the agent for spills:
    ``r_vol_spill = -4*vol_spill/vol_spill_max``
 
 2. Penalty for control effort:
    ` r_action = -0.025*(T2^2+T3^2+T6^2+T7^2) `
 
 3. Penalty for aggressive control effort:
    `r_aggressive_action = -0.01*(dT2^2+dT3^2+dT6^2+dT7^2)`
 
 4. Penalty for COM velocity:
    `rd_COM = -0.5*(dCOM_x^2+dCOM_y^2+dCOM_z^2)`
 
 6. Reward for difference COM from COM_target:
    `r_diff_COM = exp[-2000*{(COM_x-COM_target_x)^2+(COM_y-COM_target_y)^2+(COM_z-COM_target_z)^2}]`
 
 8. Liquid-Retention–Weighted Reward:
    
    `lambda = max(0, m_liquid / m_0);`
 
    `reward=lambda^3*r_diff_COM+(1+3*(1-lambda)^3)* (r_vol_spill+r_action+r_agressive_action+rd_COM).`
   
  **Main physical scheme**
 ![alt text](https://github.com/zhus-dika/sloshing_balance_via_roboarm/blob/main/rlKinova_marsRover_fluent_via_files/images/main_scheme.png)
## 🐛 About agent training
### 🐑 Randomization path
 In each episode, a random path is selected from the 28 available paths.
 <img src="https://github.com/zhus-dika/sloshing_balance_via_roboarm/blob/main/rlKinova_marsRover_fluent_via_files/images/rover_paths.png" width="600">
### 🦞 Randomization of the rover's velocity and acceleration
 The rover’s velocity and acceleration are randomized in each episode: the target speed is sampled from the interval `v_rover∼U(0.10, 0.25)[m/s]`, and the acceleration from `a_rover∼U(0.05, 0.11)[m/s²]`.
### 🐀 Randomization of the episode start time
 In each episode, the onset of RL control is randomized: the agent starts interacting with the environment after a random delay 
`t_{start}∼U(0.5, 17)[s]` from the beginning of the simulation.

To ensure stable and physically consistent behavior, we use an in-house PD controller in the Simulink model setup:

**PD controller for joint warm-up**

 For each controlled joint (R2, R3, R6, R7) we use a simple PD controller during the warm-up phase (before `rlStartTime`).  
 The goal is to keep the arm near a desired configuration and avoid large transients while the rover settles on the terrain.
 
 The holding torque is computed as:
 
 `tau_hold = K_p (q_ref - q) - K_d * q'`
 
 where:
 
 - `q_ref` — desired joint angle (initial pose)  
 - `q` — measured joint angle  
 - `q'` — measured joint angular velocity  
 - `K_p, K_d` — proportional and derivative gains  
 
 The resulting torque `tau_hold` is saturated and applied to the joint instead of the RL agent action while `t < rlStartTime`.

 When `t >= rlStartTime`, a Simulink `Switch` block routes the torque command from the RL agent, effectively disabling the PD controller during the learning/control phase.
## 🦏 Make animation 

1. 🦩 Save video with rover
   - in Mechanics Explorer as MJPEG-AVI (SIMULINK)
     ```
     mdl = 'rlKinovaBallBalance';
     
     smwritevideo(mdl,'rover_100fps.avi', ...
      'VideoFormat','motion jpeg avi', ...
      'FrameRate',100, ...
      'FrameSize',[1920 1080], ...
      'PlaybackSpeedRatio',1.0);
     ```
   - Convert avi to mp4 in bash
     ```
     ffmpeg -i data/output/rover_100fps.avi \
       -c:v libx264 -pix_fmt yuv420p -crf 16 -preset medium -r 100 -movflags +faststart \
       data/output/rover_100fps.mp4
     ```
2. 🦡 Save video with vessel
   - Save simulation frames in FLUENT

     *Solution* &rarr; *Activities* &rarr; *Create* &rarr; *Solution Animations*

   - Combine png files to video in bash
     ```
     ffmpeg -framerate 100 -start_number 0 -i sloshing_balance_via_roboarm/glass_liquid_sloshing_simulink/animation-2_%04d.png   -vf "pad=iw+mod(iw\,2):ih+mod(ih\,2):color=white"   -c:v libx264 -pix_fmt yuv420p -crf 18 -preset medium -movflags +faststart   sloshing_balance_via_roboarm/glass_liquid_sloshing_simulink/sloshing_100fps.mp4
     ```
3. 🦆 Combine 2 videos in bash
   ```
   # with TIMER
   ffmpeg -i mars_rover+kinova_roboarm_liquid-sloshing_top.mp4 -i sloshing_100fps.mp4 \
     -filter_complex "\
   [0:v]scale=-2:1080,setsar=1[base]; \
   [1:v]scale=iw/2:-2,setsar=1,format=rgba,\
   drawtext=fontfile=/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf:\
   text='Sloshing (Fluent)':x=10:y=10:fontsize=28:fontcolor=white:box=1:boxcolor=black@0.5,\
   pad=iw+8:ih+8:4:4:white[small]; \
   [base][small]overlay=20:20:shortest=1, \
   drawtext=fontfile=/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf:\
   text='t=%{pts\\:hms}':x=w-tw-20:y=h-th-20:fontsize=32:fontcolor=white:box=1:boxcolor=black@0.4" \
     -r 100 -c:v libx264 -pix_fmt yuv420p -crf 18 -preset medium -movflags +faststart \
     rover_slosh_pip_left_time_100fps.mp4
   ```
   ```
   # high quality 
   ffmpeg -i rover_100fps.mp4 -i sloshing_100fps.mp4 \
   -filter_complex "\
   [0:v]scale=-2:1080:flags=lanczos,setsar=1[base]; \
   [1:v]scale=iw/2:-2:flags=lanczos,setsar=1,format=rgba,\
   drawtext=fontfile=/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf:\
   text='Sloshing (Fluent)':x=10:y=10:fontsize=28:fontcolor=white:box=1:boxcolor=black@0.5,\
   pad=iw+8:ih+8:4:4:white[small]; \
   [base][small]overlay=20:20:shortest=1, \
   drawtext=fontfile=/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf:\
   text='t=%{pts\\:hms}':x=w-tw-20:y=h-th-20:fontsize=32:fontcolor=white:box=1:boxcolor=black@0.4,\
   format=yuv420p" \
   -c:v libx264 -preset slow -crf 17 \
   -movflags +faststart \
   rover_slosh_time_100fps_hq.mp4
   ```
   ```
   # convert video2gif
   ffmpeg -i rover_slosh_time_100fps_hq.mp4 \
     -filter_complex "[0:v]fps=30,scale=640:-1:flags=lanczos,split[v0][v1]; \
                      [v0]palettegen=stats_mode=full[pal]; \
                      [v1][pal]paletteuse=dither=bayer:bayer_scale=1:diff_mode=rectangle:new=1" \
     -loop 0 rover_slosh_time_100fps_hq.gif
   ```
   ![Alt Text](https://github.com/zhus-dika/sloshing_balance_via_roboarm/blob/main/rlKinova_marsRover_fluent_via_files/data/output/animations/rover_slosh_time_100fps_hq.gif)
   
   or in youtube https://youtu.be/L5km3YW9cCE

```sudo mkswap /swapfile2```

```sudo swapon /swapfile2```
