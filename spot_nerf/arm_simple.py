# Copyright (c) 2022 Boston Dynamics, Inc.  All rights reserved.
#
# Downloading, reproducing, distributing or otherwise using the SDK Software
# is subject to the terms and conditions of the Boston Dynamics Software
# Development Kit License (20191101-BDSDK-SL).

"""Tutorial to show how to use Spot's arm.
"""
from __future__ import print_function
import math
import argparse
import sys
import time
import cv2
import numpy as np
import bosdyn.api.gripper_command_pb2
import bosdyn.client
import bosdyn.client.lease
import bosdyn.client.util
from bosdyn.api import arm_command_pb2, geometry_pb2
from bosdyn.client import math_helpers
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME, get_a_tform_b
from bosdyn.client.robot_command import (RobotCommandBuilder, RobotCommandClient,
                                         block_until_arm_arrives, blocking_stand)
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME, VISION_FRAME_NAME, BODY_FRAME_NAME, get_se2_a_tform_b
from bosdyn.api.basic_command_pb2 import RobotCommandFeedbackStatus
from bosdyn.client.image import ImageClient, build_image_request
from bosdyn.api import image_pb2

def hello_arm(config):
    """A simple example of using the Boston Dynamics API to command Spot's arm."""

    # See hello_spot.py for an explanation of these lines.
    bosdyn.client.util.setup_logging(config.verbose)

    sdk = bosdyn.client.create_standard_sdk('HelloSpotClient')
    robot = sdk.create_robot(config.hostname)
    bosdyn.client.util.authenticate(robot)
    #image stuff
    robot.sync_with_directory()
    robot.time_sync.wait_for_sync()

    assert robot.has_arm(), "Robot requires an arm to run this example."

    # Verify the robot is not estopped and that an external application has registered and holds
    # an estop endpoint.
    assert not robot.is_estopped(), "Robot is estopped. Please use an external E-Stop client, " \
                                    "such as the estop SDK example, to configure E-Stop."

    robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
    #image client for taking image
    image_client = robot.ensure_client(ImageClient.default_service_name)


    lease_client = robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)
    with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
        # Now, we are ready to power on the robot. This call will block until the power
        # is on. Commands would fail if this did not happen. We can also check that the robot is
        # powered at any point.
        robot.logger.info("Powering on robot... This may take a several seconds.")
        robot.power_on(timeout_sec=20)
        assert robot.is_powered_on(), "Robot power on failed."
        robot.logger.info("Robot powered on.")

        # Tell the robot to stand up. The command service is used to issue commands to a robot.
        # The set of valid commands for a robot depends on hardware configuration. See
        # SpotCommandHelper for more detailed examples on command building. The robot
        # command service requires timesync between the robot and the client.
        robot.logger.info("Commanding robot to stand...")
        command_client = robot.ensure_client(RobotCommandClient.default_service_name)
        blocking_stand(command_client, timeout_sec=10)
        robot.logger.info("Robot standing.")
        start_index = 0
        #start_index = take_images(robot, robot_state_client, command_client, image_client, start_index)

        walk_to_other_side(command_client, robot_state_client)
        walk_to_other_side(command_client, robot_state_client)
        #walk_to_left_side(command_client, robot_state_client)
        #start_index = take_images(robot, robot_state_client, command_client, image_client, start_index)

        # Power the robot off. By specifying "cut_immediately=False", a safe power off command
        # is issued to the robot. This will attempt to sit the robot before powering off.
        robot.power_off(cut_immediately=False, timeout_sec=20)
        assert not robot.is_powered_on(), "Robot power off failed."
        robot.logger.info("Robot safely powered off.")

def take_images(robot, robot_state_client, command_client, image_client, start_index):
    # xyz of the robot girpper relative to the torso
    x, y, z = 0.75, 0, 0.25
    # Rotation as a quaternion
    [qx, qy, qz, qw] = euler_to_quaternion(0, 0, 0)
    # move the arm based on the position and quaternion
    arm_move(x, y, z, qw, qx, qy, qz, robot,
             robot_state_client, command_client)

    start_y, finish_y, num_shot = -0.4, 0.4, 11
    ys= np.linspace(start_y, finish_y, num_shot)
    # x, z remains as constants for now
    x, z = 0.85, -0.15
    pitch = 30
    #when arm is out, the relative distance on xaxis is around 60cm
    distance_to_bottle = 0.6

    for y in ys:
        yaw_angle = -np.arctan(y / distance_to_bottle) / np.pi * 180
        [qx, qy, qz, qw] = euler_to_quaternion(yaw_angle, pitch, 0)
        arm_move(x, y, z, qw, qx, qy, qz, robot,
                robot_state_client, command_client)
        time.sleep(3)
        take_image(image_client, name=str(start_index))
        start_index += 1

    stow = RobotCommandBuilder.arm_stow_command()

    # Issue the command via the RobotCommandClient
    stow_command_id = command_client.robot_command(stow)

    robot.logger.info("Stow command issued.")
    block_until_arm_arrives(command_client, stow_command_id, 3.0)
    return start_index

def walk_to_other_side(command_client, robot_state_client):
    try:
        # move the robot to the side
        dy = 0.5
        # 1.1 is the body length.
        distance_to_obj = 0.8
        dx = distance_to_obj * 2 + 1.1

        dyaw = 180
        relative_move(0, dy, math.radians(0), ODOM_FRAME_NAME,
                      command_client, robot_state_client)
        # move the robot forward
        relative_move(dx, 0, math.radians(0), ODOM_FRAME_NAME,
                      command_client, robot_state_client)
        # turn the robot
        relative_move(0, 0, math.radians(dyaw), ODOM_FRAME_NAME,
                      command_client, robot_state_client)
        # move the robot back to the center
        relative_move(0, dy, math.radians(0), ODOM_FRAME_NAME,
                      command_client, robot_state_client)

    finally:
        # Send a Stop at the end, regardless of what happened.
        command_client.robot_command(RobotCommandBuilder.stop_command())

def walk_to_left_side(command_client, robot_state_client):
    try:
        # move the robot to the side

        # 1.1 is the body length.
        distance_to_obj = 0.8
        dx = (distance_to_obj * 2 + 1.1)/2
        dy = dx

        dyaw = -90
        relative_move(0, dy, math.radians(0), ODOM_FRAME_NAME,
                      command_client, robot_state_client)
        # move the robot forward
        relative_move(dx, 0, math.radians(0), ODOM_FRAME_NAME,
                      command_client, robot_state_client)
        # turn the robot
        relative_move(0, 0, math.radians(dyaw), ODOM_FRAME_NAME,
                      command_client, robot_state_client)


    finally:
        # Send a Stop at the end, regardless of what happened.
        command_client.robot_command(RobotCommandBuilder.stop_command())

def block_until_arm_arrives_with_prints(robot, command_client, cmd_id):
    """Block until the arm arrives at the goal and print the distance remaining.
        Note: a version of this function is available as a helper in robot_command
        without the prints.
    """
    while True:
        feedback_resp = command_client.robot_command_feedback(cmd_id)
        robot.logger.info(
            'Distance to go: ' +
            '{:.2f} meters'.format(feedback_resp.feedback.synchronized_feedback.arm_command_feedback
                                   .arm_cartesian_feedback.measured_pos_distance_to_goal) +
            ', {:.2f} radians'.format(
                feedback_resp.feedback.synchronized_feedback.arm_command_feedback.
                arm_cartesian_feedback.measured_rot_distance_to_goal))

        if feedback_resp.feedback.synchronized_feedback.arm_command_feedback.arm_cartesian_feedback.status == arm_command_pb2.ArmCartesianCommand.Feedback.STATUS_TRAJECTORY_COMPLETE:
            robot.logger.info('Move complete.')
            break
        time.sleep(0.1)

def arm_move(x,y,z,qw,qx,qy,qz,robot,robot_state_client,command_client):
    # Move the arm to a spot in front of the robot, and open the gripper.
    # Make the arm pose RobotCommand
    # Build a position to move the arm to (in meters, relative to and expressed in the gravity aligned body frame).

    hand_ewrt_flat_body = geometry_pb2.Vec3(x=x, y=y, z=z)
    flat_body_Q_hand = geometry_pb2.Quaternion(w=qw, x=qx, y=qy, z=qz)
    flat_body_T_hand = geometry_pb2.SE3Pose(position=hand_ewrt_flat_body,
                                            rotation=flat_body_Q_hand)

    robot_state = robot_state_client.get_robot_state()
    odom_T_flat_body = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot,
                                     ODOM_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME)

    odom_T_hand = odom_T_flat_body * math_helpers.SE3Pose.from_obj(flat_body_T_hand)
    # duration in seconds
    seconds = 2
    arm_command = RobotCommandBuilder.arm_pose_command(
        odom_T_hand.x, odom_T_hand.y, odom_T_hand.z, odom_T_hand.rot.w, odom_T_hand.rot.x,
        odom_T_hand.rot.y, odom_T_hand.rot.z, ODOM_FRAME_NAME, seconds)

    # Make the open gripper RobotCommand
    gripper_command = RobotCommandBuilder.claw_gripper_open_fraction_command(1.0)

    # Combine the arm and gripper commands into one RobotCommand
    command = RobotCommandBuilder.build_synchro_command(gripper_command, arm_command)

    # Send the request
    cmd_id = command_client.robot_command(command)
    robot.logger.info('Moving arm to a position.')

    # Wait until the arm arrives at the goal.
    block_until_arm_arrives_with_prints(robot, command_client, cmd_id)

def relative_move(dx, dy, dyaw, frame_name, robot_command_client, robot_state_client, stairs=False):
    transforms = robot_state_client.get_robot_state().kinematic_state.transforms_snapshot

    # Build the transform for where we want the robot to be relative to where the body currently is.
    body_tform_goal = math_helpers.SE2Pose(x=dx, y=dy, angle=dyaw)
    # We do not want to command this goal in body frame because the body will move, thus shifting
    # our goal. Instead, we transform this offset to get the goal position in the output frame
    # (which will be either odom or vision).
    out_tform_body = get_se2_a_tform_b(transforms, frame_name, BODY_FRAME_NAME)
    out_tform_goal = out_tform_body * body_tform_goal

    # Command the robot to go to the goal point in the specified frame. The command will stop at the
    # new position.
    robot_cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
        goal_x=out_tform_goal.x, goal_y=out_tform_goal.y, goal_heading=out_tform_goal.angle,
        frame_name=frame_name, params=RobotCommandBuilder.mobility_params(stair_hint=stairs))
    end_time = 10.0
    cmd_id = robot_command_client.robot_command(lease=None, command=robot_cmd,
                                                end_time_secs=time.time() + end_time)
    # Wait until the robot has reached the goal.
    while True:
        feedback = robot_command_client.robot_command_feedback(cmd_id)
        mobility_feedback = feedback.feedback.synchronized_feedback.mobility_command_feedback
        if mobility_feedback.status != RobotCommandFeedbackStatus.STATUS_PROCESSING:
            print("Failed to reach the goal")
            return False
        traj_feedback = mobility_feedback.se2_trajectory_feedback
        if (traj_feedback.status == traj_feedback.STATUS_AT_GOAL and
                traj_feedback.body_movement_status == traj_feedback.BODY_STATUS_SETTLED):
            print("Arrived at the goal.")
            return True
        time.sleep(1)

    return True

def take_image(image_client, name):
    pixel_format = image_pb2.Image.PIXEL_FORMAT_RGB_U8
    image_request = build_image_request("hand_color_image", pixel_format=pixel_format)
    image_responses = image_client.get_image([image_request])
    for image in image_responses:
        num_bytes = 3
        dtype = np.uint8
        extension = ".jpg"
        img = np.frombuffer(image.shot.image.data, dtype=dtype)
        img = cv2.imdecode(img, -1)
        image_saved_path = name
        image_saved_path = image_saved_path.replace("/", '')
        cv2.imwrite(image_saved_path + extension, img)

def euler_to_quaternion(yaw, pitch, roll):
    yaw = degree_to_radiance(yaw)
    pitch = degree_to_radiance(pitch)
    roll = degree_to_radiance(roll)
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]

def degree_to_radiance(degree):
    return degree / 360 * 2 * np.pi

def pixel_format_string_to_enum(enum_string):
    return dict(image_pb2.Image.PixelFormat.items()).get(enum_string)

def main(argv):
    """Command line interface."""
    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_base_arguments(parser)
    options = parser.parse_args(argv)
    try:
        hello_arm(options)
        return True
    except Exception as exc:  # pylint: disable=broad-except
        logger = bosdyn.client.util.get_logger()
        logger.exception("Threw an exception")
        return False


if __name__ == '__main__':
    if not main(sys.argv[1:]):
        sys.exit(1)
