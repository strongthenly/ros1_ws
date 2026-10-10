#!/usr/bin/env python3
"""Relay simulated IMU and RGB only after calibration has been reviewed."""
import os

import rospy
import yaml
from sensor_msgs.msg import Image, Imu


REQUIRED = (
    "algorithm_imu_source",
    "lidar_imu_transform",
    "lidar_camera_transform",
    "body_lidar_transform",
    "camera_intrinsics",
    "sensor_time_offsets",
    "imu_noise",
)


def load_verified(path, expected_imu_source):
    if not os.path.isfile(path):
        raise ValueError("calibration file does not exist: " + path)
    with open(path, encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    missing = [name for name in REQUIRED if config.get(name) is None]
    if config.get("status") != "verified" or missing:
        raise ValueError("calibration is not verified; missing: " + ", ".join(missing))
    if config["algorithm_imu_source"] != expected_imu_source:
        raise ValueError(
            "IMU source mismatch: calibration=%s launch=%s"
            % (config["algorithm_imu_source"], expected_imu_source)
        )
    return config


def main():
    rospy.init_node("twin_validated_inputs")
    calibration = rospy.get_param("~calibration")
    expected_source = rospy.get_param("~expected_imu_source", "airsim_body_imu")
    try:
        load_verified(calibration, expected_source)
    except ValueError as exc:
        rospy.logfatal(str(exc))
        raise SystemExit(2)

    imu_in = rospy.get_param("~imu_in", "/twin/imu/data_raw")
    image_in = rospy.get_param("~image_in", "/twin/camera/color/image_raw")
    imu_out = rospy.get_param("~imu_out", "/livox/imu")
    image_out = rospy.get_param("~image_out", "/left_camera/image")
    imu_pub = rospy.Publisher(imu_out, Imu, queue_size=200)
    image_pub = rospy.Publisher(image_out, Image, queue_size=20)
    rospy.Subscriber(imu_in, Imu, imu_pub.publish, queue_size=200)
    rospy.Subscriber(image_in, Image, image_pub.publish, queue_size=20)
    rospy.loginfo("validated inputs active: %s -> %s; %s -> %s", imu_in, imu_out, image_in, image_out)
    rospy.spin()


if __name__ == "__main__":
    main()
