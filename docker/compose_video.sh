#!/bin/bash
# Crops the raw Gazebo / RViz recordings, puts them side by side with captions and writes
# /out/kf_vs_ekf_gazebo_rviz.mp4 (2x speed; Gazebo ran at ~0.6x real time) and a smaller GIF for the README.
set -e
F=/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf
FB=/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf
cap() { echo "drawtext=fontfile=$FB:text='$1':x=18:y=14:fontsize=26:fontcolor=white,drawtext=fontfile=$F:text='$2':x=18:y=50:fontsize=19:fontcolor=0xC8CCD4"; }
ffmpeg -y -loglevel error -i /out/gazebo.mp4 -i /out/rviz.mp4 -filter_complex "
 [0:v]crop=670:930:270:72,scale=-2:900,pad=iw:ih+84:0:84:color=0x1E2026,$(cap 'Gazebo' 'ROSbot driving a loop around the table')[g];
 [1:v]crop=928:930:16:62,scale=-2:900,pad=iw:ih+84:0:84:color=0x1E2026,$(cap 'RViz' 'white truth · grey noisy odometry · red KF · blue EKF (covariance ×30)')[r];
 [g][r]hstack,setpts=PTS/2,fps=30[v]" -map "[v]" -c:v libx264 -crf 22 -preset slow -pix_fmt yuv420p -movflags +faststart /out/kf_vs_ekf_gazebo_rviz.mp4
ffmpeg -y -loglevel error -i /out/kf_vs_ekf_gazebo_rviz.mp4 -vf "setpts=PTS/1.5,fps=12,scale=900:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=128:stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=4:diff_mode=rectangle" /out/kf_vs_ekf_gazebo_rviz.gif
ffmpeg -y -loglevel error -sseof -1 -i /out/kf_vs_ekf_gazebo_rviz.mp4 -frames:v 1 /out/kf_vs_ekf_gazebo_rviz.png
