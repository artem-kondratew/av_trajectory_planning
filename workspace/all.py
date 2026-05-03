#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt


# ======================
#   START INDICES
# ======================
STANLEY_START = 60
MPC_START = 0
PP_START = 10


def load(prefix):
    t = np.load(f"{prefix}_t.npy")
    e = np.load(f"{prefix}_e.npy")
    theta = np.load(f"{prefix}_theta.npy")
    steer = np.load(f"{prefix}_steer.npy")

    # выравниваем длины
    n = min(len(t), len(e), len(theta), len(steer))

    return (
        t[:n],
        e[:n],
        theta[:n],
        steer[:n]
    )


def apply_start(t, e, theta, steer, start_idx):
    t = t[start_idx:]
    e = e[start_idx:]
    theta = theta[start_idx:]
    steer = steer[start_idx:]

    # нормализуем время
    t = t - t[0]

    return t, e, theta, steer


def main():
    # ======================
    #   LOAD DATA
    # ======================
    stanley_t, stanley_e, stanley_theta, stanley_steer = load("stanley")
    mpc_t, mpc_e, mpc_theta, mpc_steer = load("mpc")
    pp_t, pp_e, pp_theta, pp_steer = load("pp")

    # ======================
    #   APPLY START
    # ======================
    stanley_t, stanley_e, stanley_theta, stanley_steer = apply_start(
        stanley_t, stanley_e, stanley_theta, stanley_steer, STANLEY_START
    )

    mpc_t, mpc_e, mpc_theta, mpc_steer = apply_start(
        mpc_t, mpc_e, mpc_theta, mpc_steer, MPC_START
    )

    pp_t, pp_e, pp_theta, pp_steer = apply_start(
        pp_t, pp_e, pp_theta, pp_steer, PP_START
    )

    # перевод theta в градусы
    stanley_theta = stanley_theta * 180 / np.pi
    mpc_theta = mpc_theta * 180 / np.pi
    pp_theta = pp_theta * 180 / np.pi

    # ======================
    #   PLOTS
    # ======================
    plt.figure(figsize=(12, 12))

    # --- e ---
    plt.subplot(3,1,1)
    plt.plot(stanley_t, stanley_e, label="stanley", color='r')
    plt.plot(mpc_t, mpc_e, label="mpc", color='b')
    plt.plot(pp_t, pp_e, label="pp", color='g')

    plt.ylabel('e [m]')
    plt.title('Rear axle offset')
    plt.grid()
    plt.legend()
    plt.ylim(-1.0, 1.0)

    # --- theta ---
    plt.subplot(3,1,2)
    plt.plot(stanley_t, stanley_theta, label="stanley", color='r')
    plt.plot(mpc_t, mpc_theta, label="mpc", color='b')
    plt.plot(pp_t, pp_theta, label="pp", color='g')

    plt.ylabel('theta [deg]')
    plt.title('Heading error')
    plt.grid()
    plt.legend()
    plt.ylim(-10, 10)

    # --- steer ---
    plt.subplot(3,1,3)
    plt.plot(stanley_t, stanley_steer, label="stanley", color='r')
    plt.plot(mpc_t, mpc_steer, label="mpc", color='b')
    plt.plot(pp_t, pp_steer, label="pp", color='g')

    plt.ylabel('steer [deg]')
    plt.xlabel('time [s]')
    plt.title('Steering angle')
    plt.grid()
    plt.legend()
    plt.ylim(-8, 8)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
