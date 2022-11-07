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
#include <util/util_tf.hpp>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelState.h>
#include <anno_msgs/ObjectInfo.h>

enum Mode
{
    Random,
    FullCustom
};

struct GazeboModelType
{
    gazebo_msgs::ModelState gazebo_model;
    anno_msgs::ObjectInfo object_info;
};
struct GazeboModelMultiType
{
    std::vector<gazebo_msgs::ModelState> gazebo_models;
    std::vector<anno_msgs::ObjectInfo> object_infoes;
};
class DecidePosition
{
public:
    DecidePosition();
    anno_msgs::ObjectInfo register_object(int, std::string);
    GazeboModelMultiType get_ramdam_place_position(std::vector<anno_msgs::ObjectInfo>);
    GazeboModelMultiType get_remove_position(std::vector<anno_msgs::ObjectInfo>);
    GazeboModelType get_box_position(double);
    GazeboModelType get_phoxi_position(double, double, double);
    static gazebo_msgs::ModelState make_gazebo_model_state(std::string, geometry_msgs::Transform);
    void change_mode(Mode);
    void set_parameter();
    XmlRpc::XmlRpcValue param_list;
    Mode mode_;

private:
    ros::NodeHandle pnh_;
    double z_position_;
    double box_height_;
    double object_radious_;
};