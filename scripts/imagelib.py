#!/usr/bin/env python
import rospy
import sys
from intera_core_msgs.srv._IOComponentCommandSrv import IOComponentCommandSrv
from intera_core_msgs.msg._IOComponentCommand import IOComponentCommand

def switch_camera(cam):
    if cam == 'head':
        camera_command_client(camera='right_hand_camera', status=False)
        camera_command_client(camera='head_camera', status=True)
    elif cam == 'arm':
        camera_command_client(camera='head_camera', status=False)
        camera_command_client(camera='right_hand_camera', status=True)
    else:
        raise Exception("Invalid Argument. Cam must be 'arm' or 'head'")


def camera_command_client(camera, status, timeout=0.0):
    """
    based on https://www.aransena.com/blog/2018/1/11/rethink-robotics-sawyer-camera-access
    :param camera:
    :param status:
    :param timeout:
    :return:
    """
    rospy.wait_for_service('/io/internal_camera/' + camera + '/command')
    try:
        cam_control = rospy.ServiceProxy('/io/internal_camera/' + camera + '/command', IOComponentCommandSrv)
        cmd = IOComponentCommand()
        cmd.time = rospy.Time.now()
        cmd.op = 'set'
        if status:
            cmd.args = '{"signals": {"camera_streaming": {"data": [true], "format": {"type": "bool"}}}}'
        else:
            cmd.args = '{"signals": {"camera_streaming": {"data": [false], "format": {"type": "bool"}}}}'

        resp = cam_control(cmd, timeout)
        print resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)