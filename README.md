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

1. 🦩 Save video with rover
   - in Mechanics Explorer as MJPEG-AVI (SIMULINK)
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
<<<<<<< Updated upstream
   ![Alt Text](https://github.com/zhus-dika/sloshing_balance_via_roboarm/blob/main/rlKinova_marsRover_fluent_via_files/data/output/rover-side_slosh_pip_time_100fps.gif)
=======
![Alt Text](https://github.com/zhus-dika/sloshing_balance_via_roboarm/blob/main/rlKinova_marsRover_fluent_via_files/data/output/animations/rover_slosh_time_100fps_hq.gif)
>>>>>>> Stashed changes

   or in youtube https://youtu.be/L5km3YW9cCE
### 🐠 Useful bash commands
Allocate additional swap

```sudo fallocate -l 4G /swapfile2```

```sudo chmod 600 /swapfile2```

```sudo mkswap /swapfile2```

```sudo swapon /swapfile2```
