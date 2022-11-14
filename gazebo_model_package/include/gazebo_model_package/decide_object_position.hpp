/**
 * @file decide_object_position.hpp
 * @author tsuchidashinya (tsuchida.shinya413@mail.kyutech.jp)
 * @brief
 * @version 0.1
 * @date 2022-09-10
 *
 * @copyright Copyright (c) 2022
 *
 */

#pragma once
#include <util/util_base.hpp>
#include <tf_package/tf_basic.hpp>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelState.h>
#include <anno_msgs/ObjectInfo.h>


class DecidePosition
{
public:
    DecidePosition();
    anno_msgs::ObjectInfo make_object_info(int, std::string);
    anno_msgs::ObjectInfo make_object_info(std::string, std::string);
    std::vector<anno_msgs::ObjectInfo> get_randam_place_position(std::vector<anno_msgs::ObjectInfo>);
    std::vector<anno_msgs::ObjectInfo> get_remove_position(std::vector<anno_msgs::ObjectInfo>);
    anno_msgs::ObjectInfo get_box_position();
    anno_msgs::ObjectInfo get_sensor_position();
    void set_parameter();
    XmlRpc::XmlRpcValue param_list;

private:
    ros::NodeHandle pnh_;
    std::string box_name_, sensor_name_;
    double z_position_;
    double box_height_;
    double object_radious_, object_height_;
    double sensor_angle_min_, sensor_angle_max_, sensor_distance_;
};