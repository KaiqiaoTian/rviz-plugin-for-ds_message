#!/usr/bin/env python


import rospy

from ds_av_msgs.msg import DataspeedObjectArray, DataspeedObject
from tf.transformations import *
rospy.init_node("DataspeedObjectArray_sample")
pub = rospy.Publisher("object", DataspeedObjectArray, queue_size=10)
r = rospy.Rate(24)
counter = 0
while not rospy.is_shutdown():
  box_a = DataspeedObject()
  box_b = DataspeedObject()
  box_a.label = "2"
  box_b.label = "5"
  box_a.id = 1
  box_b.id = 2
  box_arr = DataspeedObjectArray()
  now = rospy.Time.now()
  box_a.header.stamp = now
  box_b.header.stamp = now

  box_arr.header.stamp = now

  box_a.header.frame_id = "map"
  box_b.header.frame_id = "map"
  box_arr.header.frame_id = "map"
  
  q = quaternion_about_axis((counter % 100) * math.pi * 2 / 100.0, [0, 0, 1])
  box_a.pose.orientation.x = q[0]
  box_a.pose.orientation.y = q[1]
  box_a.pose.orientation.z = q[2]
  box_a.pose.orientation.w = q[3]
  box_b.pose.orientation.w = 1
  box_b.pose.position.y = 2
  box_b.bounding_box_axes.x = (counter % 10 + 1) * 0.1
  box_b.bounding_box_axes.y = ((counter + 1) % 10 + 1) * 0.1
  box_b.bounding_box_axes.z = ((counter + 2) % 10 + 1) * 0.1
  box_a.bounding_box_axes.x = 1
  box_a.bounding_box_axes.y = 1
  box_a.bounding_box_axes.z = 1

  box_a.bounding_box_offset.x = 1
  box_a.bounding_box_offset.y = 1
  box_a.bounding_box_offset.z = 1
  box_b.bounding_box_offset.x = 1
  box_b.bounding_box_offset.y = 1
  box_b.bounding_box_offset.z = 1

  box_a.velocity.linear.x = (counter % 10 + 5) *0.2
  box_a.velocity.linear.y = 0
  box_a.velocity.linear.z = 0

  box_b.velocity.linear.x = (counter % 10 + 5) * 0.2
  box_b.velocity.linear.y = 0
  box_b.velocity.linear.z = 0

  box_a.color.r = 225
  box_a.color.g = 100
  box_a.color.b = 100
  box_a.color.a = 1

  box_b.color.r = 100
  box_b.color.g = 100
  box_b.color.b = 200
  box_b.color.a = 1  

  #box_a.value = (counter % 100) / 100.0
  #box_b.value = 1 - (counter % 100) / 100.0
  box_arr.objects.append(box_a)
  box_arr.objects.append(box_b)
  pub.publish(box_arr)
  r.sleep()
  counter = counter + 1
  
