#!/usr/bin/env python3
import numpy as np
# from sympy import re
import rospy
from std_msgs.msg import ColorRGBA
from gazebo_msgs.srv import SetLightProperties, SetLightPropertiesRequest
from gazebo_msgs.srv import GetLightProperties, GetLightPropertiesRequest, GetLightPropertiesResponse
from geometry_msgs.msg import Pose, Quaternion, Vector3

class LightRandomizer:
    def __init__(self, color_range, light_name='sun', gazebo_ns='/gazebo'):
        self._light_name = light_name
        self._color_range = color_range
        self._set_light = rospy.ServiceProxy(gazebo_ns + '/set_light_properties', SetLightProperties)
        self._get_light = rospy.ServiceProxy(gazebo_ns + '/get_light_properties', GetLightProperties)

    def callback(self, event):
        req = SetLightPropertiesRequest()
        req1 = GetLightPropertiesRequest()
        req1.light_name = self._light_name
        res1 = self._get_light.call(req1)
        
        # res1 = GetLightPropertiesResponse()
        req.light_name = self._light_name
        # req.diffuse = res1.diffuse
        # req.attenuation_constant = res1.attenuation_constant
        # req.attenuation_linear = res1.attenuation_linear
        # req.attenuation_quadratic = res1.attenuation_quadratic
        req.cast_shadows = True
        difuse = ColorRGBA()
        difuse.r = float(204/255)
        difuse.g = float(204/255)
        difuse.b = float(204/255)
        difuse.a = float(255/255)
        
        attenuation_constant = 0.90
        attenuation_linear = 0.01
        attenuation_qudratic = 0.00
        req.attenuation_constant = attenuation_constant
        req.attenuation_linear = attenuation_linear
        req.attenuation_quadratic = attenuation_qudratic
        

        direction = Vector3()
        direction.x = -0.483368
        direction.y = 0.096674
        direction.z = -0.870063
        req.direction = direction

        specular = ColorRGBA()
        specular.r = float(51/255)
        specular.g = float(51/255)
        specular.b = float(51/255)
        specular.a = float(255/255)
        req.diffuse = difuse
        req.specular = specular
        pose = Pose()
        pose.position.x = 0.00
        pose.position.y = 0.00
        pose.position.z = 10.00

        pose.orientation.x = 0.00
        pose.orientation.y = 0.00
        pose.orientation.z = 0.00
        pose.orientation.w = 1.00
        req.pose = pose
        
        
        req.diffuse = ColorRGBA(*[np.random.uniform(self._color_range['r']['min'], self._color_range['r']['max']),
                                  np.random.uniform(self._color_range['g']['min'], self._color_range['g']['max']),
                                  np.random.uniform(self._color_range['b']['min'], self._color_range['b']['max']),
                                  np.random.random()])
        req.specular = ColorRGBA(*[np.random.uniform(self._color_range['r']['min'], self._color_range['r']['max']),
                                  np.random.uniform(self._color_range['g']['min'], self._color_range['g']['max']),
                                  np.random.uniform(self._color_range['b']['min'], self._color_range['b']['max']),
                                  np.random.random()])
        
        req.attenuation_constant = np.random.random()
        req.attenuation_linear = np.random.random()
        req.attenuation_quadratic = np.random.random()
        
        rospy.logdebug("Set light parameter: " + str(req))
        try:
            # res = self._get_light.call(req1)
            
            
            
            
            # print(res.attenuation_constant)
            # print(res.attenuation_linear)
            # print(res.attenuation_quadratic)
            # print(res.diffuse)
            # print("fe")
            res = self._set_light(req)
            # print("88re")
            # if not res.success:
                # rospy.logwarn(res.status_message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

if __name__ == "__main__":
    import argparse
    from std_msgs.msg import Empty
    parser = argparse.ArgumentParser(description='Light randomizer')
    parser.add_argument('-d', '--duration', type=float, default=1.0, help='Timer duration.')
    parser.add_argument('--gazebo_ns', type=str, default='/gazebo', help='Gazebo namespace.')
    parser.add_argument('-e', '--event_mode', type=str, default='timer', choices=['timer', 'trigger'], help='Timer duration.')
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("light_randomizer")
    color_range = {}
    color_range['r'] = rospy.get_param("~light_color_range/r", {'min': 0.0, 'max': 1.0})
    color_range['g'] = rospy.get_param("~light_color_range/g", {'min': 0.0, 'max': 1.0})
    color_range['b'] = rospy.get_param("~light_color_range/b", {'min': 0.0, 'max': 1.0})
    rospy.loginfo("Load param light_color_range: " + str(color_range))
    randomizer = LightRandomizer(color_range, gazebo_ns=args.gazebo_ns)
    if args.event_mode == 'timer':
        rospy.Timer(rospy.Duration(args.duration), randomizer.callback)
    elif args.event_mode == 'trigger':
        rospy.Subscriber('randomizer/trigger', Empty, randomizer.callback)
    else:
        raise ValueError('Unknown event_mode: %s' % args.event_mode)
    rospy.spin()
