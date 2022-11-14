/**
 * @file gazebo_model_server.hpp
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
#include "decide_object_position.hpp"

class GazeboModelMove
{
public:
    GazeboModelMove(ros::NodeHandle);
    static gazebo_msgs::ModelState make_gazebo_model_state(anno_msgs::ObjectInfo);
    static gazebo_msgs::ModelState make_gazebo_model_state(std::string, geometry_msgs::Transform);
    void set_multi_gazebo_model(std::vector<anno_msgs::ObjectInfo>);
    void set_gazebo_model(anno_msgs::ObjectInfo);

private:
    ros::NodeHandle nh_;
    ros::ServiceClient gazebo_client_;
    std::string gazebo_service_name_;
};
