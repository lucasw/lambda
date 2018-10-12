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
    ind = i % period
    if ind < period / 2:
        fr = float(ind) / (period / 2)
    else:
        fr = float(period - ind - period/2) / (period / 2)
    audio.data.append(fr)
pub.publish(audio)
