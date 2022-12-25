#!/usr/bin/env python3
import numpy as np
import rospy
from std_msgs.msg import ColorRGBA
from gazebo_ext_msgs.srv import GetVisualNames, GetVisualNamesRequest
from gazebo_ext_msgs.srv import SetLinkVisualProperties, SetLinkVisualPropertiesRequest
from gazebo_ext_msgs.srv import SetLinkColor, SetLinkColorRequest
from gazebo_domain_randomizer import utils

class LinkVisualPropertiesRandomizer:
    def __init__(self, model_name, color_range, gazebo_ns='/gazebo', gazebo_gui_ns='/gazebo_gui'):
        self._model_name = model_name
        self._color_range = color_range
        res = utils.get_model_properties(self._model_name, gazebo_ns)
        link_names = ["%s::%s" % (model_name, b) for b in res.body_names]
        print(link_names)
        rospy.wait_for_service(gazebo_ns + '/get_visual_names')
        get_vis_names = rospy.ServiceProxy(gazebo_ns + '/get_visual_names', GetVisualNames)
        try:
            res = get_vis_names(GetVisualNamesRequest(link_names=link_names))
            if not res.success:
                rospy.logwarn(res.status_message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
        self._visuals = res.link_visual_names
        self._parents = res.link_parent_names
        self._set_link_vis = rospy.ServiceProxy(gazebo_ns + '/set_link_color', SetLinkColor)

    def callback(self, event):
        if len(self._visuals) == 0:
            return
        req = SetLinkColorRequest()
        idx = np.random.choice(len(self._visuals))
        req.link_visual_name = self._visuals[idx]
        req.link_parent_name = self._parents[idx]
        color= ColorRGBA(*[np.random.uniform(self._color_range['r']['min'], self._color_range['r']['max']),
                           np.random.uniform(self._color_range['g']['min'], self._color_range['g']['max']),
                           np.random.uniform(self._color_range['b']['min'], self._color_range['b']['max']),
                           1.0])
        req.ambient = color
        req.diffuse = color
        req.emissive = ColorRGBA(0.0, 0.0, 0.0, 1.0)
        try:
            res = self._set_link_vis(req)
            if not res.success:
                rospy.logwarn(res.status_message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

if __name__ == "__main__":
    import argparse
    from std_msgs.msg import Empty
    parser = argparse.ArgumentParser(description='Link visual properties randomizer')
    parser.add_argument('-m', '--model_name', type=str, default='', help='Model name.')
    parser.add_argument('-d', '--duration', type=float, default=1.0, help='Timer duration.')
    parser.add_argument('--gazebo_ns', type=str, default='/gazebo', help='Gazebo namespace.')
    parser.add_argument('--gazebo_gui_ns', type=str, default='/gazebo_gui', help='Gazebo gui namespace.')
    parser.add_argument('-e', '--event_mode', type=str, default='timer', choices=['timer', 'trigger'], help='Timer duration.')
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("link_visual_properties_randomizer")
    color_range = {}
    color_range['r'] = rospy.get_param("~link_color_range/r", {'min': 0.0, 'max': 1.0})
    color_range['g'] = rospy.get_param("~link_color_range/g", {'min': 0.0, 'max': 1.0})
    color_range['b'] = rospy.get_param("~link_color_range/b", {'min': 0.0, 'max': 1.0})
    rospy.loginfo("Load param link_color_range: " + str(color_range))
    randomizer = LinkVisualPropertiesRandomizer(args.model_name, color_range,
                                                gazebo_ns=args.gazebo_ns, gazebo_gui_ns=args.gazebo_gui_ns)
    if args.event_mode == 'timer':
        rospy.Timer(rospy.Duration(args.duration), randomizer.callback)
    elif args.event_mode == 'trigger':
        rospy.Subscriber('randomizer/trigger', Empty, randomizer.callback)
    else:
        raise ValueError('Unknown event_mode: %s' % args.event_mode)
    rospy.spin()