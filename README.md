# Kalman Filter vs. Extended Kalman Filter for a Differential-Drive Robot

![KF vs EKF: true vs. estimated trajectory with covariance ellipses](docs/kf_vs_ekf.gif)

A **Kalman filter (KF)** and an **extended Kalman filter (EKF)** written in C++ (ROS 1 + Eigen) that estimate
the pose `(x, y, θ)` of a differential-drive robot from its velocity commands and noisy odometry.
In the original setup the robot is a Husarion ROSbot driven by keyboard teleop in a Gazebo house world;
the filtered pose and its covariance are published as `PoseWithCovarianceStamped` and shown live in RViz.

> **About the animation:** ROS 1 / Gazebo is not available on the machine this README was prepared on, so the GIF
> above comes from [`sim/kf_ekf_sim.py`](sim/kf_ekf_sim.py), a line-by-line **NumPy port of `KF.cpp` and `EKF.cpp`**
> (same matrices, noise values, `dt = 0.02 s` and start pose). Only the environment is replaced: a scripted velocity
> command drives a unicycle model with wheel slip, and the "odometry" is the true pose plus Gaussian noise.

## The model

**State and input**

```
x_t = [x, y, θ]ᵀ          u = (v, ω)   from /cmd_vel
z_t = [x, y, θ]ᵀ + δ,  δ ~ N(0, Q)     from /rosbot/odom (yaw from the quaternion)
```

**Motion model (unicycle, Euler step Δt = 0.02 s)**

```
x_t = x_{t-1} + Δt · v · cos θ_{t-1}
y_t = y_{t-1} + Δt · v · sin θ_{t-1}
θ_t = θ_{t-1} + Δt · ω
```

Process noise enters on the inputs, `ε ~ N(0, R)` with `R = diag(0.01, 0.01)`, mapped into state space by

```
      ⎡Δt·cos θ   0 ⎤
G_u = ⎢Δt·sin θ   0 ⎥
      ⎣   0      Δt ⎦
```

`R` is gated to zero when the measured velocities are zero, so the covariance does not grow while the robot stands still.
Measurement noise `Q = 0.01·I₃`, observation matrix `C = H = I₃`.

### KF — linear form

The nonlinearity is pushed into the input vector, giving a linear system with `A = I`, `B = Δt·I`:

```
u_KF = [v cos θ̂, v sin θ̂, ω]ᵀ
x̄ = A x̂ + B u_KF                  Σ̄ = A Σ Aᵀ + G_u R G_uᵀ
```

### EKF — linearised with the Jacobian

```
x̄ = g(x̂, u)                       Σ̄ = G_x Σ G_xᵀ + G_u R G_uᵀ

      ⎡1  0  −Δt·v·sin θ⎤
G_x = ⎢0  1   Δt·v·cos θ⎥   = ∂g/∂x
      ⎣0  0       1     ⎦
```

The Jacobians `G_x` and `G_u` were derived symbolically in MATLAB ([`matlab/KF_and_EKF.m`](matlab/KF_and_EKF.m)).

### Correction (both filters)

```
K  = Σ̄ Cᵀ (C Σ̄ Cᵀ + Q)⁻¹
x̂  = x̄ + K (z − C x̄)
Σ  = (I − K C) Σ̄
```

[`matlab/Covariance_Matrix_limit.m`](matlab/Covariance_Matrix_limit.m) iterates the covariance recursion to check
that Σ converges to a steady state instead of growing without bound.

## Results

![KF and EKF side by side](docs/kf_vs_ekf.png)

From the offline re-run (32 s, seed 4):

| | position RMSE |
|---|---|
| raw odometry `z` | 14.0 cm |
| KF | 1.4 cm |
| EKF | 1.4 cm |

Both filters cut the position error by about 10× compared with the raw measurement. With the KF written this way
(heading folded into `u`) the **predicted mean is the same as in the EKF**; the difference is in the
**covariance**. With `A = I` the KF ignores how heading uncertainty turns into lateral position uncertainty, so its
ellipses stay narrow across the direction of travel. The EKF's `G_x` couples θ into x/y, so its ellipses grow
sideways in a more realistic way. That consistency matters once the measurements get sparse or worse.

## Repository layout

```
ros_ws/src/gruppe2_bringup/   ROS 1 catkin package (package name kept from the course)
  src/KF.{hpp,cpp}            Kalman filter class
  src/EKF.{hpp,cpp}           Extended Kalman filter class
  src/*_node.cpp              nodes: subscribe /rosbot/cmd_vel + /rosbot/odom, publish KF_Position / EKF_Position
  launch/bringup_{KF,EKF}.launch   Gazebo + ROSbot + teleop + filter node + RViz
  rviz/, urdf/, meshes/, worlds/
matlab/                       symbolic Jacobian derivation and covariance-limit check
sim/kf_ekf_sim.py             offline NumPy re-run that generates the figures
docs/                         GIF and PNG
```

## Build and run

**ROS (built with ROS Noetic, Ubuntu 20.04)**

```bash
sudo apt install ros-noetic-desktop-full ros-noetic-turtlebot3-teleop ros-noetic-topic-tools libeigen3-dev xterm
cd ros_ws
catkin build            # or catkin_make
source devel/setup.bash
roslaunch gruppe2_bringup bringup_KF.launch    # or bringup_EKF.launch
```

Drive the robot with the keyboard in the teleop xterm. `topic_relay.sh` forwards `/cmd_vel` to `/rosbot/cmd_vel`.
The filter node prints the estimate, measurement and Σ, and RViz shows the estimate with its covariance.

**Offline re-run (no ROS needed)**

```bash
pip install numpy matplotlib pillow
python sim/kf_ekf_sim.py
```

## Context

Solo project for the master's course MUS at FH Technikum Wien, 2021.

For this portfolio version the vendored Eigen copy was replaced by the system Eigen (`find_package(Eigen3)`), and
build output and generated Doxygen HTML were removed. The filter code is unchanged.

**Third-party assets:** the ROSbot URDF and meshes come from Husarion's `rosbot_description`, and `house.world` comes
from ROBOTIS' `turtlebot3_gazebo`. Both stay under their original licences. Everything else is MIT, see [LICENSE](LICENSE).
