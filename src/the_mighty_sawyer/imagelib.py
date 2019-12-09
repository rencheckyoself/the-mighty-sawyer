#!/usr/bin/env python

"""
Provides functions for visual aspects of project (cameras, display images, etc).

"""
import rospy
import sys
from intera_core_msgs.srv._IOComponentCommandSrv import IOComponentCommandSrv
from intera_core_msgs.msg._IOComponentCommand import IOComponentCommand
import cv2
import numpy as np
import os
import intera_interface, intera_io


def switch_camera(cam):
    """
    Change active camera feed on Sawyer
    args- cam (arm, head)
    """

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
    client to process Sawyer camera commands
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


def put_text(text, bg_color = (255,255,255), text_color = (0,0,0), scale=3, thickness=4):
    """
    Write text to Sawyer's display screen

    get emojis here: https://emojiisland.com/pages/free-download-emoji-icons-png

    :param text:  string- test to displya
    :param bg_color:  (b,g,r)- background color
    :param text_color:  (b,g,r)- text color
    :param scale:  float- font size
    :param thickness:  float- font thickness
    :return:
    """

    disp = intera_interface.HeadDisplay()

    img = np.zeros(shape=[600, 1024, 3], dtype=np.uint8)

    b, g, r = bg_color
    img[:,:,0] = b
    img[:, :, 1] = g
    img[:, :, 2] = r

    font = cv2.FONT_HERSHEY_DUPLEX

    textsize = cv2.getTextSize(text, font, scale, thickness)[0]

    textX = (img.shape[1] - textsize[0]) / 2
    textY = (img.shape[0] + textsize[1]) / 2

    img = cv2.putText(img, text, (textX, textY), font, scale, text_color, thickness)

    cv2.imwrite('temp.png', img)
    disp.display_image('temp.png')
    os.remove('temp.png')

def display_png(img_name, bg_color=(255,255,255)):
    """
    Displays a png image from imagaes/display directory on Sawyer's screen.
    """
    disp = intera_interface.HeadDisplay()

    img = np.zeros(shape=[600, 1024, 3], dtype=np.uint8)

    b, g, r = bg_color
    img[:,:,0] = b
    img[:, :, 1] = g
    img[:, :, 2] = r

    # sloppy way to get paths to display images- don't use but may be useful in other nodes
    img_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    img_dir = os.path.join(img_dir, 'images', 'display')
    img_path = os.path.join(img_dir, img_name)
    pic = cv2.imread(img_path, cv2.IMREAD_UNCHANGED)

    alpha = pic[:, :, 3]
    h, w = alpha.shape

    for x in range(h):
        for y in range(w):
            if not alpha[x, y]:
                pic[x, y] = (b, g, r, 1)

    offh = int((img.shape[0] - pic.shape[0]) / 2)
    offw = int((img.shape[1] - pic.shape[1]) / 2)

    img[offh:(offh + pic.shape[0]), offw:(offw + pic.shape[1]), :] = pic[:, :, :3]

    cv2.imwrite('temp.png', img)
    disp.display_image('temp.png')
    os.remove('temp.png')


def set_exposure(cam, val):
    """
    sets exposure of selected camera
    """
    if cam == 'arm':
        cam = 'right_hand_camera'
    elif cam == 'head':
        cam = 'head_camera'

    io = intera_io.IODeviceInterface('internal_camera', cam)
    io.set_signal_value('set_exposure', val)


#
# def display_image(img):
#     """
#     Displays an image from imagaes/display directory on Sawyer's screen.
#     TODO need to test if __file__ path is library or node calling library
#     """
#     disp = intera_interface.HeadDisplay()
#
#     # sloppy way to get paths to display images- don't use but may be useful in other nodes
#     img_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
#     img_dir = os.path.join(img_dir, 'images', 'display')
#     disp.display_image(os.path.join(img_dir, 'blank_screen.png'))
