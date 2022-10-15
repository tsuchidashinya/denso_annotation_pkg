/**
 * @file decide_object_position.cpp
 * @author tsuchidashinya (tsuchida.shinya413@mail.kyutech.jp)
 * @brief
 * @version 0.1
 * @date 2022-09-11
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <gazebo_model_package/decide_object_position.hpp>

DecideObjectPosition::DecideObjectPosition()
    : pnh_("~")
{
    pnh_.getParam("decide_object_position", param_list);
    mode_ = Mode::FullAuto;
}

/**
 * @brief オブジェクトの配置を決定する
 *
 * @return std::vector<anno_msgs::ObjectInfo>
 */
std::vector<anno_msgs::ObjectInfo> DecideObjectPosition::decide_object_position()
{
    std::vector<anno_msgs::ObjectInfo> output;
    output.resize(static_cast<int>(param_list["the_number_of_object"]));
    for (int i = 0; i < output.size(); i++)
    {
    }
}

/**
 * @brief gazebo_modelメッセージを生成する関数です。
 *
 * @param object_name Gazeboオブジェクトの名前
 * @param trans geometry_msgs::Transform型の姿勢データ
 * @return gazebo_msgs::ModelState
 */
gazebo_msgs::ModelState DecideObjectPosition::gazebo_model_state_make(std::string object_name, geometry_msgs::Transform trans)
{
    gazebo_msgs::ModelState model;
    model.model_name = object_name;
    model.pose.position.x = trans.translation.x;
    model.pose.position.y = trans.translation.y;
    model.pose.position.z = trans.translation.z;
    tf2::convert(trans.rotation, model.pose.orientation);
    return model;
}
