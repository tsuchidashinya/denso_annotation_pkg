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
    set_parameter();
}

void DecidePosition::set_parameter()
{
    z_position_ = param_list["z_position"];
    box_height_ = param_list["box_height"];
    box_name_ = static_cast<std::string>(param_list["box_name"]);
    object_radious_ = param_list["radious"];
}


anno_msgs::ObjectInfo DecidePosition::make_object_info(int object_id, std::string object_name)
{
    anno_msgs::ObjectInfo output;
    output.tf_name = object_name + "_" + std::to_string(object_id);
    output.object_name = object_name;
    return output;
}
/**
 * @brief オブジェクトの配置を決定する
 *
 * @return std::vector<anno_msgs::ObjectInfo>
 */
std::vector<anno_msgs::ObjectInfo> DecidePosition::get_randam_place_position(std::vector<anno_msgs::ObjectInfo> object_info)
{
    GazeboModelMultiType output;

    {
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
            object_info[i].position = TfBasic::make_geo_transform(x, y, z, TfBasic::rotate_xyz_make(0, 0, 0));
        }
    }
    return object_info;
}

std::vector<anno_msgs::ObjectInfo> DecidePosition::get_remove_position(std::vector<anno_msgs::ObjectInfo> object_info)
{
    for (int i = 0; i < object_info.size(); i++)
    {
        double x = 100;
        double y = 100;
        double z = 100;
        tf2::Quaternion quaternion = TfBasic::rotate_xyz_make(0, 0, 0);
        object_info[i].position =  TfBasic::make_geo_transform(x, y, z, quaternion);
    }
    return object_info;
}

// anno_msgs::ObjectInfo DecidePosition::get_box_position(double probability)
// {
//     UtilBase util;
//     GazeboModelType output;
//     if (probability <= util.random_float(0, 1))
//     {
//         double x = 0;
//         double y = 0;
//         double z = z_position_;
//         tf2::Quaternion quat = TfBasic::rotate_xyz_make(0, 0, 0);
//         output.object_info.position = UtilBase::geo_trans_make(x, y, z, quat);
//         output.object_info.object_name = "denso_box";
//         output.gazebo_model = make_gazebo_model_state(output.object_info.object_name, output.object_info.position);
//     }
//     else
//     {
//         double x = 100;
//         double y = 100;
//         double z = z_position_;
//         tf2::Quaternion quat = TfBasic::rotate_xyz_make(0, 0, 0);
//         output.object_info.position = UtilBase::geo_trans_make(x, y, z, quat);
//         output.object_info.object_name = "denso_box";
//         output.gazebo_model = make_gazebo_model_state(output.object_info.object_name, output.object_info.position);
//     }
//     return output;
// }

anno_msgs::ObjectInfo DecidePosition::get_sensor_position()
{
    UtilBase util;
    GazeboModelType output;
    double angle = util.random_float(angle_min, angle_max);
    double x = distance * sin(angle);
    double y = 0;
    double z = distance * cos(angle);
    tf2::Quaternion quaternion = TfBasic::rotate_xyz_make(0, angle, 0);
    output.object_info.position = UtilBase::geo_trans_make(x, y, z, quaternion);
    output.object_info.object_name = "phoxi_camera";
    output.gazebo_model = make_gazebo_model_state(output.object_info.object_name, output.object_info.position);
    return output;
}

