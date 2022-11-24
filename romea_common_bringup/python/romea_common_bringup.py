#!/usr/bin/env python3


def robot_prefix(robot_name):

    if robot_name == "":
        return "/"
    else:
        return "/" + robot_name + "/"


def device_prefix(robot_name, device_name):
    return robot_prefix(robot_name) + device_name + "/"


def robot_urdf_prefix(robot_name):

    if robot_name == "" :
      return ""
    else:
     return robot_name+"_"
