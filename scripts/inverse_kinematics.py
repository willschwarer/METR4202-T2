"""
Name: Chris Smith
Student Number: 44803625
Course: METR4202 - UQ
Assignment: Team Project
Doc: 4R Gripper Inverse Kinematics
"""

import numpy as np

def IKin_3R(L1, L2, L3_x, L3_y, x, y):
    """
    Parameters
    ----------
    L1 : Float
        Length of link 1.
    L2 : Float
        Length of link 2.
    L3_x : Float
        Length of gripper horizontal offset.
    L3_y : Float
        Length of gripper vertical offset.
    x : Float
        End effecter x coordinate.
    y : Float
        End effecter y coordinate.

    Returns
    -------
    theta_1_down: Float
        Angle in radians of L1 from the x-axis with elbow joint down.
    theta_2_down: Float
        Angle in radians of L2 from L1 with elbow joint down.
    theta_3_down: Float
        Angle in radians of L3 from L2 with elbow joint down.
    theta_1_up: Float
        Angle in radians of L1 from the x-axis with elbow joint up.
    theta_2_up: Float
        Angle in radians of L2 from L1 with elbow joint up.
    theta_3_up: Float
        Angle in radians of L3 from L2 with elbow joint up.
    """

    # Step 1: Calculating the coordinates of the wrist joint
    if x > (L1 + L2) * 0.9:
        x_wrist = x - L3_y
        y_wrist = y - L3_x
    else:
        x_wrist = x - L3_x
        y_wrist = y + L3_y

    # Step 2: Calculating the internal elbow angles alpha, beta, and gamma

    # alpha describes the angle of the wrist joint from the x-axis
    if x_wrist == 0:
        alpha = np.radians(90)
    else:
        alpha = np.arctan(y_wrist / x_wrist)

    # beta describes the angle from L2 to L1
    beta = np.arccos((L1**2 + L2**2 - x_wrist**2 - y_wrist**2) / (2 * L1 * L2))

    # gamma describes the angle from L1 to the wrist joint
    gamma = np.arccos((x_wrist**2 + y_wrist**2 + L1**2 - L2**2) / (2 * L1 * np.sqrt(x_wrist**2 + y_wrist**2)))

    # Step 3: Calculating the final joint angles
    theta_1_down = alpha - gamma
    theta_2_down = np.pi - beta

    if x > (L1 + L2) * 0.9:
        theta_3_down = np.radians(0) - (theta_1_down + theta_2_down)
    else:
        theta_3_down = np.radians(-90) - (theta_1_down + theta_2_down)

    theta_1_up = theta_1_down + 2 * gamma
    theta_2_up = -1 * theta_2_down
    theta_3_up = theta_3_down + 2 * theta_2_down - 2 * gamma

    return theta_1_down, theta_2_down, theta_3_down, theta_1_up, theta_2_up, theta_3_up

def cartesian_to_polar(x, y):
    """
    Parameters
    ----------
    x : Float
        x cartestian coordinate.
    y : Float
        y cartesian coordinate.

    Returns
    -------
    r : Float
        Length polar coordinate.
    theta : Float
        Angle polar coordinate in radians.
    """

    r = np.sqrt(x**2 + y**2)

    # Handle the divide 0 case
    if y == 0 and x < 0:
        theta = np.radians(-90)
    elif y == 0 and x >= 0:
        theta = np.radians(90)
    else:
        theta = -np.arctan(x / y)

    return r, theta

def IKin_4R(L0, L1, L2, L3_x, L3_y, x, y, z):
    """
    Parameters
    ----------
    L0 : Float
        Length of link 0.
    L1 : Float
        Length of link 1.
    L2 : TYPE
        Length of link 2.
    L3_x : Float
        Length of gripper horizontal offset.
    L3_y : Float
        Length of gripper vertical offset.
    x : Float    theta_0 -= np.radians(90)
        Cartesian x coordinate of desired end effecter location in {s}.
    y : Float
        Cartesian y coordinate of desired end effecter location in {s}.
    z : Float
        Cartesian z coordinate of desired end effecter location in {s}.

    Returns
    -------
    theta_0 : Float
        Angle in radians of the decoupled 3R from the y-axis.
    theta_1_down: Float
        Angle in radians of L1 from the x-axis with elbow joint down.
    theta_2_down: Float
        Angle in radians of L2 from L1 with elbow joint down.
    theta_3_down: Float
        Angle in radians of L3 from L2 with elbow joint down.
    theta_1_up: Float
        Angle in radians of L1 from the x-axis with elbow joint up.
    theta_2_up: Float
        Angle in radians of L2 from L1 with elbow joint up.
    theta_3_up: Float
        Angle in radians of L3 from L2 with elbow joint up.
    """

    r, theta_0 = cartesian_to_polar(x, y)

    # Calibration offset for physical system
    z += 20

    theta_1_down, theta_2_down, theta_3_down, theta_1_up, theta_2_up, theta_3_up = \
        IKin_3R(L1, L2, L3_x, L3_y, r, z - L0)

    return theta_0, theta_1_down, theta_2_down, theta_3_down, theta_1_up, theta_2_up, theta_3_up

def main():
    L0, L1, L2, L3_x, L3_y = 100, 117.5, 95, 15, 95

    x, y, z = 0, 240, 40

    theta_0, theta_1_down, theta_2_down, theta_3_down, theta_1_up, theta_2_up, theta_3_up = \
        IKin_4R(L0, L1, L2, L3_x, L3_y, x, y, z)

    # print('Joint Angles:\n\n',
    #       'theta 4 =', round(np.degrees(theta_0), 3), 'deg\n',
    #       'theta 3 =', round(np.degrees(theta_1_up), 3), 'deg\n',
    #       'theta 2 =', round(np.degrees(theta_2_up), 3), 'deg\n',
    #       'theta 1 =', round(np.degrees(theta_3_up), 3), 'deg\n')

    print('Joint Angles:\n\n',
          'theta 4 =', round(theta_0, 3), 'rad\n',
          'theta 3 =', round(np.radians(90) - theta_1_up, 3), 'rad\n',
          'theta 2 =', round(-theta_2_up, 3), 'rad\n',
          'theta 1 =', round(theta_3_up, 3), 'rad\n')

if __name__ == '__main__':
    main()
