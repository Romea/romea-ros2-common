#!/usr/bin/env python3


def robot_prefix(robot_namespace):

    if robot_namespace == "":
        return "/"
    else:
        return "/" + robot_namespace + "/"


def device_prefix(robot_namespace, device_name):
    return robot_prefix(robot_namespace) + device_name + "/"


def robot_urdf_prefix(robot_namespace):

    if robot_namespace == "" :
      return ""
    else:
     return robot_namespace+"_"
