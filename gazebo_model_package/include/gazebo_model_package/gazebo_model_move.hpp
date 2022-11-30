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
#include <util/util.hpp>
#include "decide_object_position.hpp"
#include <util/util_msg_data.hpp>

class GazeboMoveServer
{
public:
    GazeboMoveServer(ros::NodeHandle);
   
    void set_multi_gazebo_model(std::vector<common_msgs::ObjectInfo>);
    void set_gazebo_model(common_msgs::ObjectInfo);
    void set_parameter();
private:
    ros::NodeHandle nh_, pnh_;
    ros::Publisher gazebo_pub_;
    std::string world_frame_;
    XmlRpc::XmlRpcValue param_list;
    TfFunction tf_func_;
};
