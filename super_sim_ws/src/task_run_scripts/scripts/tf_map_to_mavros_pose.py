#!/usr/bin/env python3
"""
tf_map_to_mavros_pose.py

Lookup transform from `map` -> `odin1_base_link` and publish as
`geometry_msgs/PoseStamped` to `/mavros/vision_pose/pose`.

Usage:
  rosrun task_run_scripts tf_map_to_mavros_pose.py

Parameters (private/ns):
  ~base_frame (default: odin1_base_link)
  ~map_frame  (default: map)
  ~pub_topic  (default: /mavros/vision_pose/pose)
  ~rate       (default: 20.0)
"""
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped


def lookup_and_publish():
    rospy.init_node('tf_map_to_mavros_pose', anonymous=True)

    base_frame = rospy.get_param('~base_frame', 'odin1_base_link')
    map_frame = rospy.get_param('~map_frame', 'map')
    pub_topic = rospy.get_param('~pub_topic', '/mavros/vision_pose/pose')
    rate_hz = float(rospy.get_param('~rate', 20.0))

    pub = rospy.Publisher(pub_topic, PoseStamped, queue_size=10)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rate = rospy.Rate(rate_hz)
    first_success = False

    rospy.loginfo('tf_map_to_mavros_pose started: map_frame=%s base_frame=%s -> pub=%s @%0.1fHz',
                  map_frame, base_frame, pub_topic, rate_hz)

    while not rospy.is_shutdown():
        try:
            # lookup latest transform from map -> base_frame
            trans = tf_buffer.lookup_transform(map_frame, base_frame, rospy.Time(0), rospy.Duration(0.5))

            pose = PoseStamped()
            pose.header.frame_id = map_frame
            pose.header.stamp = rospy.Time.now()

            t = trans.transform.translation
            r = trans.transform.rotation

            pose.pose.position.x = t.x
            pose.pose.position.y = t.y
            pose.pose.position.z = t.z

            pose.pose.orientation.x = r.x
            pose.pose.orientation.y = r.y
            pose.pose.orientation.z = r.z
            pose.pose.orientation.w = r.w

            pub.publish(pose)

            if not first_success:
                rospy.loginfo('Successfully published transform %s -> %s', map_frame, base_frame)
                first_success = True

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(5, 'TF lookup failed: %s', str(e))
            first_success = False

        except Exception as e:
            rospy.logerr('Unexpected error in tf_map_to_mavros_pose: %s', str(e))

        rate.sleep()


if __name__ == '__main__':
    try:
        lookup_and_publish()
    except rospy.ROSInterruptException:
        pass
