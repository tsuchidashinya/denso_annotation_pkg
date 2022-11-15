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
    set_parameter();
}

void DecidePosition::set_parameter()
{
    pnh_.getParam("decide_object_position", param_list);
    z_position_ = param_list["z_position"];
    box_height_ = param_list["box_height"];
    box_name_ = static_cast<std::string>(param_list["box_name"]);
    object_radious_ = param_list["radious"];
    sensor_name_ = static_cast<std::string>(param_list["sensor_name"]);
    sensor_angle_max_ = param_list["sensor_angle_max"];
    sensor_angle_min_ = param_list["sensor_angle_min"];
    sensor_distance_ = param_list["distance"];
    object_height_ = param_list["object_height"];
}


anno_msgs::ObjectInfo DecidePosition::make_object_info(int object_id, std::string object_name)
{
    anno_msgs::ObjectInfo outdata;
    outdata.tf_name = object_name + "_" + std::to_string(object_id);
    outdata.object_name = object_name;
    return outdata;
}

anno_msgs::ObjectInfo DecidePosition::make_object_info(std::string object_name, std::string tf_name)
{
    anno_msgs::ObjectInfo outdata;
    outdata.object_name = object_name;
    outdata.tf_name = tf_name;
    return outdata;
}

/**
 * @brief オブジェクトの配置を決定する
 *
 * @return std::vector<anno_msgs::ObjectInfo>
 */
std::vector<anno_msgs::ObjectInfo> DecidePosition::get_randam_place_position(std::vector<anno_msgs::ObjectInfo> object_info)
{
    {
        int count = 0;
        int map_index = 0;
        std::map<int, int> check_point;
        check_point[map_index] = 0;
        bool loop_ok;
        double x, y, x_range = 0.08, y_range = 0.12;
        UtilBase util;
        for (int i = 0; i < object_info.size(); i++)
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
                    if (count == 2000)
                    {
                        ROS_WARN_STREAM("Over");
                        break;
                    }
                    // ros_print_parameter("loop_ok");
                    x = util.random_float(-x_range, x_range);
                    y = util.random_float(-y_range, y_range);

                    for (int j = check_point[map_index]; j < i; j++)
                    {
                        if (UtilBase::distance(x, y, object_info[j].position.translation.x, object_info[j].position.translation.y) < object_radious_ * 2)
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
    UtilBase util;
    for (int i = 0; i < object_info.size(); i++)
    {
        double x = util.random_float(10, 20);
        double y = util.random_float(10, 20);
        double z = util.random_float(0, 20);
        tf2::Quaternion quaternion = TfBasic::rotate_xyz_make(0, 0, 0);
        object_info[i].position =  TfBasic::make_geo_transform(x, y, z, quaternion);
    }
    return object_info;
}

anno_msgs::ObjectInfo DecidePosition::get_box_position()
{
    UtilBase util;
    anno_msgs::ObjectInfo outdata;
    double x = 0;
    double y = 0;
    double z = z_position_;
    tf2::Quaternion quat = TfBasic::rotate_xyz_make(0, 0, 0);
    outdata.position = TfBasic::make_geo_transform(x, y, z, quat);
    outdata.object_name = box_name_;
    outdata.tf_name = box_name_;
    return outdata;
}

anno_msgs::ObjectInfo DecidePosition::get_sensor_position()
{
    UtilBase util;
    anno_msgs::ObjectInfo outdata;
    double angle = util.random_float(sensor_angle_min_, sensor_angle_max_);
    double x = sensor_distance_ * sin(angle);
    double y = 0;
    double z = sensor_distance_ * cos(angle);
    tf2::Quaternion quaternion = TfBasic::rotate_xyz_make(0, angle, 0);
    outdata.position = TfBasic::make_geo_transform(x, y, z, quaternion);
    outdata.object_name = sensor_name_;
    outdata.tf_name = sensor_name_;
    return outdata;
}

