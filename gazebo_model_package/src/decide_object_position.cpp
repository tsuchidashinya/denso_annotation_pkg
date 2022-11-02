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

DecidePosition::DecidePosition()
    : pnh_("~")
{
    pnh_.getParam("decide_object_position", param_list);
    mode_ = Mode::Random;
    change_mode(mode_);
    set_parameter();
}

void DecidePosition::set_parameter()
{
    z_position_ = param_list["z_position"];
    box_height_ = param_list["box_height"];
    object_radious_ = param_list["radious"];
}

void DecidePosition::change_mode(Mode mode)
{
    mode_ = mode;
}

anno_msgs::ObjectInfo DecidePosition::register_object(int object_id, std::string object_name)
{
    anno_msgs::ObjectInfo output;
    output.object_id = object_id;
    output.object_name = object_name;
    return output;
}
/**
 * @brief オブジェクトの配置を決定する
 *
 * @return std::vector<anno_msgs::ObjectInfo>
 */
GazeboModelMultiType DecidePosition::get_object_place_position(std::vector<anno_msgs::ObjectInfo> object_info)
{
    GazeboModelMultiType output;
    if (mode_ == Mode::Random)
    {
        output.object_infoes.resize(object_info.size());
        output.gazebo_models.resize(object_info.size());
        int count = 0;
        int map_index = 0;
        std::map<int, int> check_point;
        check_point[map_index] = 0;
        bool loop_ok;
        double x, y, x_range = 0.08, y_range = 0.12;
        UtilBase util;
        for (int i = 0; i < output.object_infoes.size(); i++)
        {
            count = 0;
            if (i > 0)
            {
                if (i % 4 == 0)
                {
                    map_index++;
                    check_point[map_index] = i;
                }
                do
                {
                    loop_ok = true;

                    count++;
                    if (count == 1000)
                    {
                        break;
                    }
                    // ros_print_parameter("loop_ok");
                    x = util.random_float(-x_range, x_range);
                    y = util.random_float(-y_range, y_range);

                    for (int j = check_point[map_index]; j < i; j++)
                    {
                        double d1[2] = {x, y}, d2[2] = {output.object_infoes[j].position.translation.x, output.object_infoes[j].position.translation.y};
                        if (UtilBase::distance(d1, d2) < object_radious_ * 2)
                            loop_ok = false;
                    }

                } while (!loop_ok);
            }
            else
            {
                x = util.random_float(-x_range, x_range);
                y = util.random_float(-y_range, y_range);
            }

            double z = z_position_ + box_height_ + map_index * 0.05;
            output.object_infoes[i].position = UtilBase::geo_trans_make(x, y, z, UtilBase::rotate_xyz_make(0, 0, 0));
            output.object_infoes[i].object_id = object_info[i].object_id;
            output.object_infoes[i].object_name = object_info[i].object_name;
            output.gazebo_models[i] = make_gazebo_model_state(output.object_infoes[i].object_name, output.object_infoes[i].position);
        }
    }
    return output;
}

GazeboModelMultiType DecidePosition::get_object_remove_position(std::vector<anno_msgs::ObjectInfo> object_info)
{
    GazeboModelMultiType output;
    output.object_infoes = object_info;
    output.gazebo_models.resize(object_info.size());
    for (int i = 0; i < object_info.size(); i++)
    {
        double x = 100;
        double y = 100;
        double z = 100;
        tf2::Quaternion quaternion = UtilBase::rotate_xyz_make(0, 0, 0);
        output.gazebo_models[i] = make_gazebo_model_state(object_info[i].object_name, UtilBase::geo_trans_make(x, y, z, quaternion));
    }
    return output;
}

GazeboModelType DecidePosition::get_box_position(double probability)
{
    UtilBase util;
    GazeboModelType output;
    if (probability <= util.random_float(0, 1))
    {
        double x = 0;
        double y = 0;
        double z = z_position_;
        tf2::Quaternion quat = UtilBase::rotate_xyz_make(0, 0, 0);
        output.object_info.position = UtilBase::geo_trans_make(x, y, z, quat);
        output.object_info.object_name = "denso_box";
        output.gazebo_model = make_gazebo_model_state(output.object_info.object_name, output.object_info.position);
    }
    else
    {
        double x = 100;
        double y = 100;
        double z = z_position_;
        tf2::Quaternion quat = UtilBase::rotate_xyz_make(0, 0, 0);
        output.object_info.position = UtilBase::geo_trans_make(x, y, z, quat);
        output.object_info.object_name = "denso_box";
        output.gazebo_model = make_gazebo_model_state(output.object_info.object_name, output.object_info.position);
    }
    return output;
}

GazeboModelType DecidePosition::get_phoxi_position(double angle_min, double angle_max, double distance)
{
    UtilBase util;
    GazeboModelType output;
    double angle = util.random_float(angle_min, angle_max);
    double x = distance * sin(angle);
    double y = 0;
    double z = distance * cos(angle);
    tf2::Quaternion quaternion = UtilBase::rotate_xyz_make(0, angle, 0);
    output.object_info.position = UtilBase::geo_trans_make(x, y, z, quaternion);
    output.object_info.object_name = "phoxi_camera";
    output.gazebo_model = make_gazebo_model_state(output.object_info.object_name, output.object_info.position);
    return output;
}

/**
 * @brief gazebo_modelメッセージを生成する関数です。
 *
 * @param object_name Gazeboオブジェクトの名前
 * @param trans geometry_msgs::Transform型の姿勢データ
 * @return gazebo_msgs::ModelState
 */
gazebo_msgs::ModelState DecidePosition::make_gazebo_model_state(std::string object_name, geometry_msgs::Transform trans)
{
    gazebo_msgs::ModelState model;
    model.model_name = object_name;
    model.pose.position.x = trans.translation.x;
    model.pose.position.y = trans.translation.y;
    model.pose.position.z = trans.translation.z;
    tf2::convert(trans.rotation, model.pose.orientation);
    return model;
}
