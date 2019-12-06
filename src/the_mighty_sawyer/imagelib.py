#!/usr/bin/env python
import rospy
import sys
from intera_core_msgs.srv._IOComponentCommandSrv import IOComponentCommandSrv
from intera_core_msgs.msg._IOComponentCommand import IOComponentCommand
import cv2
import numpy as np
import os
import intera_interface



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


def put_text(text, bg_color = (255,255,255), text_color = (0,0,0), scale=5, thickness=4):
    disp = intera_interface.HeadDisplay()

    img = np.zeros(shape=[600, 1024, 3], dtype=np.uint8)

    b, g, r = bg_color
    img[:,:,0] = b
    img[:, :, 1] = g
    img[:, :, 2] = r

    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 5
    thick = 3

    textsize = cv2.getTextSize(text, font, scale, thick)[0]

    textX = (img.shape[1] - textsize[0]) / 2
    textY = (img.shape[0] + textsize[1]) / 2

    img = cv2.putText(img, text, (textX, textY), font, scale, text_color, thick)

    cv2.imwrite('temp.png', img)
    disp.display_image('temp.png')
    os.remove('temp.png')


# # create blank image - y, x
# img = np.zeros((600, 1000, 3), np.uint8)
#
# # setup text
# font = cv2.FONT_HERSHEY_SIMPLEX
# text = "Hello Joseph!!"
#
# # get boundary of this text
# textsize = cv2.getTextSize(text, font, 1, 2)[0]
#
# # get coords based on boundary
# textX = (img.shape[1] - textsize[0]) / 2
# textY = (img.shape[0] + textsize[1]) / 2
#
# # add text centered on image
# cv2.putText(img, text, (textX, textY ), font, 1, (255, 255, 255), 2)
#
# # display image
# cv2.imshow('image', img)
#
# # wait so you can see the image
# sleep(25)
#
# # cleanup
# cv2.destroyAllWindows()