#!/bin/bash
# Runs demo_kf_vs_ekf.launch headless and records Gazebo and RViz, each on its own 960x1080 virtual screen,
# then joins them side by side into /out/demo_raw.mp4.
source /ros_ws/devel/setup.bash
export GAZEBO_MODEL_DATABASE_URI="" DISABLE_ROS1_EOL_WARNINGS=1
export ROS_HOSTNAME=localhost ROS_MASTER_URI=http://localhost:11311   # works with --network none
DURATION=${DURATION:-90}
Xvfb :99 -screen 0 960x1080x24 &   # Gazebo client
Xvfb :98 -screen 0 960x1080x24 &   # RViz
sleep 2
# same world, Gazebo GUI camera moved to an angled view over the start room
sed 's#<pose>0 0 17 3.14159 1.57079 3.14159</pose>#<pose>-2.3 -0.9 4.4 0 0.91 1.5708</pose>#' \n  "$(rospack find gruppe2_bringup)/worlds/house.world" > /tmp/house_demo.world
mkdir -p ~/.gazebo && printf '[geometry]\nx=0\ny=0\nwidth=960\nheight=1080\n' > ~/.gazebo/gui.ini

DISPLAY=:99 roslaunch gruppe2_bringup demo_kf_vs_ekf.launch rviz:=false gazebo_gui:=false world:=/tmp/house_demo.world start_delay:=100000 > /out/roslaunch.log 2>&1 &
LP=$!
until timeout 5 rostopic echo -n 1 /rosbot/odom > /dev/null 2>&1; do
  kill -0 $LP 2>/dev/null || { echo "roslaunch died"; exit 1; }
  sleep 2
done
echo "robot spawned"
DISPLAY=:99 gzclient > /out/gzclient.log 2>&1 &
DISPLAY=:98 rosrun rviz rviz -d "$(rospack find gruppe2_bringup)/rviz/demo_kf_vs_ekf.rviz" > /out/rviz.log 2>&1 &
sleep 20
sleep 5

ffmpeg -y -loglevel error -f x11grab -draw_mouse 0 -framerate 15 -video_size 960x1080 -i :99 -t "$DURATION" -c:v libx264 -preset veryfast -pix_fmt yuv420p /tmp/gazebo.mp4 &
P1=$!
ffmpeg -y -loglevel error -f x11grab -draw_mouse 0 -framerate 15 -video_size 960x1080 -i :98 -t "$DURATION" -c:v libx264 -preset veryfast -pix_fmt yuv420p /tmp/rviz.mp4 &
P2=$!
sleep 1
rosparam set /demo/start true
wait $P1 $P2
ffmpeg -y -loglevel error -i /tmp/gazebo.mp4 -i /tmp/rviz.mp4 -filter_complex hstack -c:v libx264 -crf 20 -pix_fmt yuv420p /out/demo_raw.mp4
cp /tmp/gazebo.mp4 /tmp/rviz.mp4 /out/
echo done
