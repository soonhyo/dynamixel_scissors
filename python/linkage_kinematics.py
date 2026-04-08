#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
4-bar linkage kinematics for scissors mechanism.
Computes actual scissors blade angle from Dynamixel motor angle.

Linkage (XZ plane):
    O1 (motor) --[crank 2.2cm]-- A --[coupler 6.7cm]-- B --[rocker 6.7cm]-- O2 (pivot)
         |                                                       |
         +----------------- ground 7.07cm ----------------------+
"""

import numpy as np

# Fixed pivot positions (XZ plane -> 2D: x=X, y=Z)
O1 = np.array([0.01, -0.070])   # Motor shaft position
O2 = np.array([0.0, 0.0])       # Scissors pivot

# Link lengths (meters)
A1 = np.linalg.norm(O2 - O1)    # ground:  ~7.07cm
A2 = 0.022                       # crank:    2.2cm
A3 = 0.067                       # coupler:  6.7cm
A4 = 0.067                       # rocker:   6.7cm

# Ground link angle (O1 -> O2 direction)
THETA1 = np.arctan2(O2[1] - O1[1], O2[0] - O1[0])


def motor_to_scissors_angle(motor_angle):
    """
    Given motor (Dynamixel) angle [rad],
    return scissors blade angle [rad] via 4-bar linkage.

    Returns (angle_solution1, angle_solution2) or None if no solution.
    The two solutions correspond to two possible assembly modes.
    """
    # Crank tip (point A)
    A = O1 + A2 * np.array([np.cos(motor_angle), np.sin(motor_angle)])

    # Find point B: intersection of two circles
    #   Circle 1: center=A,  radius=A3 (coupler)
    #   Circle 2: center=O2, radius=A4 (rocker)
    d = np.linalg.norm(A - O2)

    if d > A3 + A4 or d < abs(A3 - A4):
        return None  # no valid assembly

    # Angle at O2 in triangle O2-A-B
    cos_alpha = (A4**2 + d**2 - A3**2) / (2.0 * A4 * d)
    cos_alpha = np.clip(cos_alpha, -1.0, 1.0)
    alpha = np.arccos(cos_alpha)

    # Direction from O2 to A
    phi = np.arctan2(A[1] - O2[1], A[0] - O2[0])

    # Two assembly modes
    theta4_1 = phi + alpha
    theta4_2 = phi - alpha

    return theta4_1, theta4_2


def print_mapping():
    """Print motor angle -> scissors angle mapping."""
    print("=" * 60)
    print(f"Ground link:  {A1*100:.2f} cm")
    print(f"Crank:        {A2*100:.1f} cm")
    print(f"Coupler:      {A3*100:.1f} cm")
    print(f"Rocker:       {A4*100:.1f} cm")
    print("=" * 60)
    print(f"{'Motor [rad]':>12s}  {'Motor [deg]':>12s}  "
          f"{'Scissors1 [deg]':>16s}  {'Scissors2 [deg]':>16s}")
    print("-" * 60)

    motor_angles = np.linspace(0.50, -3.14, 37)  # ~0.1 rad steps
    for ma in motor_angles:
        result = motor_to_scissors_angle(ma)
        if result is not None:
            s1, s2 = result
            print(f"{ma:12.3f}  {np.degrees(ma):12.1f}  "
                  f"{np.degrees(s1):16.1f}  {np.degrees(s2):16.1f}")
        else:
            print(f"{ma:12.3f}  {np.degrees(ma):12.1f}  "
                  f"{'--- no solution ---':>36s}")


def plot_mapping():
    """Plot motor angle vs scissors angle."""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not available, skipping plot")
        return

    motor_angles = np.linspace(0.50, -3.14, 200)
    s1_list, s2_list, ma_list = [], [], []

    for ma in motor_angles:
        result = motor_to_scissors_angle(ma)
        if result is not None:
            ma_list.append(np.degrees(ma))
            s1_list.append(np.degrees(result[0]))
            s2_list.append(np.degrees(result[1]))

    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    # Angle mapping
    axes[0].plot(ma_list, s1_list, 'b-', label='Assembly mode 1')
    axes[0].plot(ma_list, s2_list, 'r--', label='Assembly mode 2')
    axes[0].set_xlabel('Motor angle [deg]')
    axes[0].set_ylabel('Scissors blade angle [deg]')
    axes[0].set_title('Motor angle -> Scissors angle')
    axes[0].legend()
    axes[0].grid(True)

    # Mechanism visualization at a few positions
    ax = axes[1]
    ax.set_aspect('equal')
    ax.set_title('Linkage positions')
    ax.grid(True)

    colors = ['green', 'blue', 'orange', 'red']
    sample_angles = [0.50, -0.50, -1.50, -3.14]

    for ma, color in zip(sample_angles, colors):
        A = O1 + A2 * np.array([np.cos(ma), np.sin(ma)])
        result = motor_to_scissors_angle(ma)
        if result is None:
            continue
        theta4 = result[1]  # pick one assembly mode
        B = O2 + A4 * np.array([np.cos(theta4), np.sin(theta4)])

        # Draw linkage
        ax.plot([O1[0]*100, A[0]*100], [O1[1]*100, A[1]*100],
                '-o', color=color, markersize=4, linewidth=2,
                label=f'motor={ma:.2f} rad')
        ax.plot([A[0]*100, B[0]*100], [A[1]*100, B[1]*100],
                '-o', color=color, markersize=4, linewidth=1.5)
        ax.plot([O2[0]*100, B[0]*100], [O2[1]*100, B[1]*100],
                '-o', color=color, markersize=4, linewidth=2)

    # Ground pivots
    ax.plot(*O1*100, 'ks', markersize=8, label='Motor (O1)')
    ax.plot(*O2*100, 'k^', markersize=8, label='Pivot (O2)')
    ax.set_xlabel('X [cm]')
    ax.set_ylabel('Z [cm]')
    ax.legend(fontsize=7)

    plt.tight_layout()
    plt.savefig('/tmp/scissors_linkage.png', dpi=150)
    plt.show()
    print("Plot saved to /tmp/scissors_linkage.png")


if __name__ == '__main__':
    print_mapping()
    print()
    plot_mapping()
