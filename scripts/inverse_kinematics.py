
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
        # Gripper at 30 degrees from ground
        x_wrist = x - L3_x * np.sin(np.radians(30)) - L3_y * np.cos(np.radians(30))
        y_wrist = y - L3_x * np.cos(np.radians(30)) + L3_y * np.sin(np.radians(30))

    else:
        # Gripper vertical to ground
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
        # Gripper at 30 degrees from ground
        theta_3_down = np.radians(-30) - (theta_1_down + theta_2_down)
    else:
        # Gripper vertical from ground
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

    theta = np.arctan2(y, x) - np.radians(90)

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
    if r >= 180 and r < 192:
        z += 15
    elif r >= 192 and r < 200:
        z += 10
        r += 8
    elif r >= 200:
        z += -0.05 * r + 30
        r += 8

    theta_1_down, theta_2_down, theta_3_down, theta_1_up, theta_2_up, theta_3_up = \
        IKin_3R(L1, L2, L3_x, L3_y, r, z - L0)

    return theta_0, theta_1_down, theta_2_down, theta_3_down, theta_1_up, theta_2_up, theta_3_up

def main():
    L0, L1, L2, L3_x, L3_y = 100, 117.5, 95, 15, 105

    x, y, z = 0, 240, 20

    theta_0, theta_1_down, theta_2_down, theta_3_down, theta_1_up, theta_2_up, theta_3_up = \
        IKin_4R(L0, L1, L2, L3_x, L3_y, x, y, z)

    # Joint Angle Adjustments for 4R Robot Motor Alignments

    # print('Analytical Joint Angles :\n\n',
    #       'Joint 4 =', round(np.degrees(-theta_0), 3), 'deg\n',
    #       'Joint 3 =', round(np.degrees(np.radians(90) - theta_1_up), 3), 'deg\n',
    #       'Joint 2 =', round(np.degrees(-theta_2_up), 3), 'deg\n',
    #       'Joint 1 =', round(np.degrees(theta_3_up), 3), 'deg\n')

    # print('Joint Angles:\n\n',
    #       'Joint 4 =', round(-theta_0, 2), 'rad\n',
    #       'Joint 3 =', round(np.radians(90) - theta_1_up, 2), 'rad\n',
    #       'Joint 2 =', round(-theta_2_up, 2), 'rad\n',
    #       'Joint 1 =', round(theta_3_up, 2), 'rad\n')

if __name__ == '__main__':
    main()
