#!/usr/bin/env python
# coding: utf-8

import numpy as np
from vedo import Plotter, Box, Axes, Cylinder, Sphere, settings
from robot3D_basic import createCoordinateFrameMesh, getLocalFrameMatrix, RotationMatrix
from robot3D_basic_solution import forward_kinematics

def visualize_robot_arm(T_01, T_02, T_03, T_04, L1, L2, L3, r=0.4, plotter=None):
    """Visualize the robot arm with given transformation matrices and link lengths."""
    #  Check if a Plotter is passed, clear previous actors
    if plotter is not None:
        plotter.clear()
    else:
        plotter = Plotter()

    #  Start taking graphic data from main program
    box_mesh = Box(pos=(3, 0.7, 0), length=2, height=2, c="cyan", alpha=1)
    axes = Axes(xrange=(0, 25), yrange=(-2, 12), zrange=(0, 6))

    # Create the coordinate frame mesh and transform
    Frame1Arrows = createCoordinateFrameMesh()
    link1_mesh = Cylinder(r=r, height=L1, pos=((L1 / 2) + r, 0, 0), c="yellow", alpha=.8, axis=(1, 0, 0))

    # Also create a sphere to show as an example of a joint
    sphere1 = Sphere(r=r).pos(0, 0, 0).color("gray").alpha(.8)

    # Combine all parts into a single object
    Frame1 = Frame1Arrows + link1_mesh + sphere1

    # Transform the part to position it at its correct location and orientation
    Frame1.apply_transform(T_01)

    # Create the coordinate frame mesh and transform
    Frame2Arrows = createCoordinateFrameMesh()
    link2_mesh = Cylinder(r=r, height=L2, pos=((L2 / 2) + r, 0, 0), c="red", alpha=.8, axis=(1, 0, 0))

    # Also create a sphere to show as an example of a joint
    sphere2 = Sphere(r=r).pos(0, 0, 0).color("gray").alpha(.8)

    # Combine all parts into a single object
    Frame2 = Frame2Arrows + link2_mesh + sphere2

    # Transform the part to position it at its correct location and orientation
    Frame2.apply_transform(T_02)

    # Create the coordinate frame mesh and transform.
    Frame3Arrows = createCoordinateFrameMesh()
    link3_mesh = Cylinder(r=r, height=L3, pos=((L3 / 2) + r, 0, 0), c="green", alpha=.8, axis=(1, 0, 0))

    # Also create a sphere to show as an example of a joint
    sphere3 = Sphere(r=r).pos(0, 0, 0).color("gray").alpha(.8)

    # Combine all parts into a single object
    Frame3 = link3_mesh + Frame3Arrows + sphere3

    # Transform the part to position it at its correct location and orientation
    Frame3.apply_transform(T_03)

    # Create end-effector for end of robot arm
    end_effector = createCoordinateFrameMesh()

    # Create a sphere to show as an example of the end of the joint
    sphere4 = Sphere(r=r).pos(0, 0, 0).color("black").alpha(0.8)

    # Combine all parts into a single object
    end_effector += sphere4

    # Transform the part to position it at its correct location and orientation
    end_effector.apply_transform(T_04)

    # Set up plotter with information for show
    if plotter is not None:
        plotter.show([box_mesh, Frame1, Frame2, Frame3, end_effector, axes], viewup="xyz", interactive=False)
    return plotter

def main():
    plotter = Plotter(interactive=False)
    L1, L2, L3, L4 = [5, 8, 3, 0]  # Lengths of the robot arm segments
    Phi = np.array([30, -15, 15, 0])  # Initial joint angles

    # Animation parameters
    start_angle = 30
    end_angle = 100
    frames = 100

    for frame in range(frames):
        Phi[0] = start_angle + (end_angle - start_angle) * frame / (frames)  # Update the first joint angle
        Phi[1] = start_angle - (end_angle + start_angle) * frame / (frames - 1)  # Update the first joint angle
        Phi[2] = start_angle - (end_angle + start_angle) * frame / (frames - 1)  # Update the first joint angle

        # Calculate forward kinematics
        T_01, T_02, T_03, T_04, e = forward_kinematics(Phi, L1, L2, L3, L4)

        # Visualize the robot arm with updated positions
        plotter = visualize_robot_arm(T_01, T_02, T_03, T_04, L1, L2, L3, plotter=plotter)

    # To keep the window open at the end of the animation
    plotter.interactive = True
    plotter.show()

if __name__ == '__main__':
    main()
