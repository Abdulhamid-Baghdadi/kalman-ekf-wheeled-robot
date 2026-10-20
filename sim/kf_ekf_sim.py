"""Offline re-run of the ROS KF / EKF nodes on a simulated wheeled robot.

This is a faithful NumPy port of ros_ws/src/gruppe2_bringup/src/KF.cpp and EKF.cpp
(same state, matrices, noise values, dt and start pose). Only the environment is
replaced: instead of Gazebo + teleop, a scripted velocity command drives a
unicycle model with process noise, and the "odometry" measurement is the true
pose plus Gaussian noise.

    python sim/kf_ekf_sim.py          -> docs/kf_vs_ekf.gif, docs/kf_vs_ekf.png
"""
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Ellipse

DT = 0.02                      # same as the C++ nodes
X0 = np.array([-2.0, 1.0, 0.0])  # starting pose used in KF.cpp / EKF.cpp and the launch files
Q = np.eye(3) * 0.01           # measurement noise covariance (both filters)
R_GAIN = 0.01                  # process noise on (v, omega) while the robot moves
MEAS_STD = np.sqrt(0.01)       # simulated odometry noise, matches Q
SLIP_STD = np.array([0.05, 0.05])  # simulated wheel slip on (v, omega)
OUT = Path(__file__).resolve().parent.parent / "docs"


def command(t):
    """Scripted teleop: straight, left arc, straight, right arc, ... (v [m/s], omega [rad/s])."""
    phase = int(t // 4) % 4
    return [(0.4, 0.0), (0.35, 0.6), (0.4, 0.0), (0.35, -0.6)][phase]


def G_u(theta):
    return np.array([[DT * np.cos(theta), 0], [DT * np.sin(theta), 0], [0, DT]])


def gated_R(v_meas, w_meas):
    return np.diag([R_GAIN * (abs(v_meas) > 1e-3), R_GAIN * (abs(w_meas) > 1e-3)])


class KF:
    """Linear KF: x_t = A x + B u with u = [v cos(theta), v sin(theta), omega], A = I."""

    def __init__(self):
        self.x, self.S = X0.copy(), np.zeros((3, 3))

    def step(self, v, w, z, v_meas, w_meas):
        if abs(v_meas) <= 1e-3 and abs(w_meas) <= 1e-3:
            return
        th = self.x[2]
        u = np.array([v * np.cos(th), v * np.sin(th), w])
        A, B, C = np.eye(3), np.eye(3) * DT, np.eye(3)
        Gu, R = G_u(th), gated_R(v_meas, w_meas)
        xp = A @ self.x + B @ u
        Sp = A @ self.S @ A.T + Gu @ R @ Gu.T
        K = Sp @ C.T @ np.linalg.inv(C @ Sp @ C.T + Q)
        self.x = xp + K @ (z - C @ xp)
        self.S = (np.eye(3) - K @ C) @ Sp


class EKF:
    """EKF: nonlinear unicycle motion model, linearised with the Jacobian G_x."""

    def __init__(self):
        self.x, self.S = X0.copy(), np.zeros((3, 3))

    def step(self, v, w, z, v_meas, w_meas):
        if abs(v_meas) <= 1e-3 and abs(w_meas) <= 1e-3:
            return
        th = self.x[2]
        Gx = np.array([[1, 0, -DT * v * np.sin(th)], [0, 1, DT * v * np.cos(th)], [0, 0, 1]])
        Gu, R = G_u(th), gated_R(v_meas, w_meas)
        xp = self.x + np.array([DT * v * np.cos(th), DT * v * np.sin(th), DT * w])
        Sp = Gx @ self.S @ Gx.T + Gu @ R @ Gu.T
        K = Sp @ np.linalg.inv(Sp + Q)  # H = I
        self.x = xp + K @ (z - xp)
        self.S = (np.eye(3) - K) @ Sp


def simulate(seconds=32.0, seed=4):
    rng = np.random.default_rng(seed)
    n = int(seconds / DT)
    truth, meas = np.zeros((n, 3)), np.zeros((n, 3))
    est = {"KF": np.zeros((n, 3)), "EKF": np.zeros((n, 3))}
    cov = {"KF": np.zeros((n, 3, 3)), "EKF": np.zeros((n, 3, 3))}
    filters = {"KF": KF(), "EKF": EKF()}
    x = X0.copy()
    for k in range(n):
        v, w = command(k * DT)
        va, wa = np.array([v, w]) + rng.normal(0, SLIP_STD)
        x = x + np.array([DT * va * np.cos(x[2]), DT * va * np.sin(x[2]), DT * wa])
        z = x + rng.normal(0, MEAS_STD, 3)
        truth[k], meas[k] = x, z
        for name, f in filters.items():
            f.step(v, w, z, va, wa)
            est[name][k], cov[name][k] = f.x, f.S
    return truth, meas, est, cov


def ellipse(mean, S, color, nsig=3.0):
    vals, vecs = np.linalg.eigh(S[:2, :2])
    vals = np.clip(vals, 0, None)
    ang = np.degrees(np.arctan2(vecs[1, 1], vecs[0, 1]))
    return Ellipse(mean[:2], 2 * nsig * np.sqrt(vals[1]), 2 * nsig * np.sqrt(vals[0]),
                   angle=ang, fill=False, color=color, lw=1.2)


COL = {"KF": "#d1495b", "EKF": "#2e86ab"}


def main():
    OUT.mkdir(exist_ok=True)
    truth, meas, est, cov = simulate()
    n = len(truth)
    rmse = {k: np.sqrt(np.mean(np.sum((e[:, :2] - truth[:, :2]) ** 2, axis=1))) for k, e in est.items()}
    rmse["odom"] = np.sqrt(np.mean(np.sum((meas[:, :2] - truth[:, :2]) ** 2, axis=1)))
    print("position RMSE [m]:", {k: round(float(v), 4) for k, v in rmse.items()})

    # static figure ---------------------------------------------------------
    fig, axes = plt.subplots(1, 2, figsize=(12, 5.2))
    for ax, name in zip(axes, ["KF", "EKF"]):
        ax.plot(meas[::5, 0], meas[::5, 1], ".", color="0.75", ms=3, label="noisy odometry z")
        ax.plot(truth[:, 0], truth[:, 1], "k-", lw=1.5, label="true pose")
        ax.plot(est[name][:, 0], est[name][:, 1], "-", color=COL[name], lw=1.5, label=f"{name} estimate")
        for k in range(0, n, 100):
            ax.add_patch(ellipse(est[name][k], cov[name][k], COL[name], nsig=10))
        ax.set_title(f"{name} — position RMSE {rmse[name] * 100:.1f} cm (odometry {rmse['odom'] * 100:.1f} cm)")
        ax.set_aspect("equal"); ax.grid(alpha=0.3); ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
        ax.legend(loc="lower right", fontsize=8)
    fig.suptitle("Wheeled robot: true vs. estimated pose (ellipses = 10σ position covariance)")
    fig.tight_layout(); fig.savefig(OUT / "kf_vs_ekf.png", dpi=120); plt.close(fig)

    # animation -------------------------------------------------------------
    fig, ax = plt.subplots(figsize=(7, 5.6))
    ax.set_aspect("equal"); ax.grid(alpha=0.3)
    pad = 0.4
    ax.set_xlim(truth[:, 0].min() - pad, truth[:, 0].max() + pad)
    ax.set_ylim(truth[:, 1].min() - pad, truth[:, 1].max() + pad)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    (lz,) = ax.plot([], [], ".", color="0.75", ms=3, label="noisy odometry z")
    (lt,) = ax.plot([], [], "k-", lw=1.5, label="true pose")
    lines = {k: ax.plot([], [], "-", color=COL[k], lw=1.4, label=f"{k} estimate")[0] for k in COL}
    ells = {k: ax.add_patch(ellipse(X0, np.eye(3) * 1e-9, COL[k], nsig=10)) for k in COL}
    title = ax.set_title("")
    ax.legend(loc="upper left", fontsize=8)
    frames = range(0, n, 10)

    def draw(k):
        lz.set_data(meas[:k + 1:5, 0], meas[:k + 1:5, 1])
        lt.set_data(truth[:k + 1, 0], truth[:k + 1, 1])
        for name in COL:
            lines[name].set_data(est[name][:k + 1, 0], est[name][:k + 1, 1])
            e = ellipse(est[name][k], cov[name][k], COL[name], nsig=10)
            ells[name].set_center(e.center); ells[name].width = e.width
            ells[name].height = e.height; ells[name].angle = e.angle
        title.set_text(f"KF vs EKF, t = {k * DT:5.1f} s   (ellipses: 10σ)")
        return [lz, lt, title, *lines.values(), *ells.values()]

    FuncAnimation(fig, draw, frames=frames, blit=False).save(OUT / "kf_vs_ekf.gif", writer=PillowWriter(fps=20))
    plt.close(fig)


if __name__ == "__main__":
    main()
