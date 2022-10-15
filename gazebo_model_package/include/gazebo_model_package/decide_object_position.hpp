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
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelState.h>
#include <anno_msgs/ObjectInfo.h>

enum Mode
{
    FullAuto,
    MaxNumDecide,
    ObjectNameSpecify,
    FullCustom
};

class DecideObjectPosition
{
public:
    DecideObjectPosition();
    std::vector<anno_msgs::ObjectInfo> decide_object_position();
    static gazebo_msgs::ModelState gazebo_model_state_make(std::string, geometry_msgs::Transform);
    XmlRpc::XmlRpcValue param_list;

private:
    ros::NodeHandle pnh_;
    int mode_;
    int max_num_;
};