#!/usr/bin/env python

import math
import rospy

from spectrogram_paint_ros.msg import Audio

rospy.init_node("sine")

pub = rospy.Publisher("/lambda/audio_source", Audio, queue_size=3)
rospy.sleep(0.5)

audio = Audio()

num = rospy.get_param("~length", 100)
period = rospy.get_param("~period", 6.0)
for i in range(num):
    fr = math.sin(i * math.pi * 0.5 / num)
    val = math.sin(i * math.pi / period) * fr
    audio.data.append(val)
pub.publish(audio)
