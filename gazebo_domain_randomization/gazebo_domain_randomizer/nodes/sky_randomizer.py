#!/usr/bin/env python3
import numpy as np
import rospy
from gazebo_ext_msgs.srv import GetSkyProperties
from gazebo_ext_msgs.srv import SetSkyProperties, SetSkyPropertiesRequest

class SkyRandomizer:
    def __init__(self,  gazebo_ns='/gazebo_gui'):
        rospy.wait_for_service(gazebo_ns + '/get_sky_properties')
        get_sky = rospy.ServiceProxy(gazebo_ns + '/get_sky_properties', GetSkyProperties)
        self._set_sky = rospy.ServiceProxy(gazebo_ns + '/set_sky_properties', SetSkyProperties)
        try:
            res = get_sky()
            if not res.success:
                rospy.logwarn(res.status_message)
            else:
                self._default_props = res
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
	    

    def callback(self, event):
        req = SetSkyPropertiesRequest()
        req.time = np.random.uniform(0, 24)
        req.sunrise = self._default_props.sunrise
        req.sunset = self._default_props.sunset
        req.wind_speed = self._default_props.wind_speed
        req.wind_direction = self._default_props.wind_direction
        req.cloud_ambient = self._default_props.cloud_ambient
        req.humidity = self._default_props.humidity
        req.mean_cloud_size = self._default_props.mean_cloud_size
        rospy.logdebug("Set sky parameter: " + str(req))
        try:
            res = self._set_sky(req)
            if not res.success:
                rospy.logwarn(res.status_message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

if __name__ == "__main__":
    import argparse
    from std_msgs.msg import Empty
    parser = argparse.ArgumentParser(description='Sky randomizer')
    parser.add_argument('-d', '--duration', type=float, default=1.0, help='Timer duration.')
    parser.add_argument('--gazebo_ns', type=str, default='/gazebo_gui', help='Gazebo namespace.')
    parser.add_argument('-e', '--event_mode', type=str, default='timer', choices=['timer', 'trigger'], help='Timer duration.')
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("sky_randomizer")
    randomizer = SkyRandomizer(gazebo_ns=args.gazebo_ns)
    if args.event_mode == 'timer':
        rospy.Timer(rospy.Duration(args.duration), randomizer.callback)
    elif args.event_mode == 'trigger':
        rospy.Subscriber('randomizer/trigger', Empty, randomizer.callback)
    else:
        raise ValueError('Unknown event_mode: %s' % args.event_mode)
    rospy.spin()
