#!/usr/bin/env python3
import os, sys
from common_msgs.msg import CloudData
from anno_srvs.srv import RecordService, RecordServiceResponse
import rospy
import rosparam
from util import util_python
from annotation_common import hdf5_function
from tqdm import tqdm


class RecordServiceClass():
    def __init__(self):
        rospy.init_node('record_service')
        self.set_parameter()
        rospy.Service(self.hdf5_record_service_name, RecordService, self.hdf5_service_callback)

    def hdf5_service_callback(self, request):
        if request.current_index == 0:
            self.bar = tqdm(total=request.the_number_of_dataset)
            self.bar.set_description("Progress rate")
            self.hdf5_object = hdf5_function.open_writed_hdf5(util_python.decide_allpath(self.hdf5_file_dir, self.hdf5_file_name))
        if request.the_number_of_dataset == request.current_index + 1:
            hdf5_function.close_hdf5(self.hdf5_object)
            hdf5_function.concatenate_hdf5(self.hdf5_file_dir, util_python.decide_allpath(self.hdf5_file_dir, self.hdf5_file_name))
        index = request.current_index % self.hdf5_save_interval
        if index == 0:
            hdf5_function.close_hdf5(self.hdf5_object)
            self.hdf5_object = hdf5_function.open_writed_hdf5(util_python.decide_allpath(self.hdf5_file_dir, self.hdf5_file_name))
        np_cam = util_python.make_npcam_from_roscam(request.camera_info)
        np_img = util_python.make_npimg_from_rosimg(request.image)
        np_cloud = util_python.make_npcloud_from_cloud(request.cloud_data)
        np_cloud, np_mask = util_python.extract_mask_from_npcloud(np_cloud)
        translation, rotation = util_python.make_pose_data(request.pose_datas)
        pose_mask = util_python.make_pose_mask(translation, rotation)
        if request.annotation_task == "Acc":    
            data_dict = {"Points": np_cloud, "masks": np_mask, "translation": translation,
            "rotation": rotation, "image": np_img, "camera_info": np_cam}
        elif request.annotation_task == "segmentation":
            data_dict = {"Points": np_cloud, "masks": np_mask}
        elif request.annotation_task == "clustering":
            data_dict = {"pcl": np_cloud, "class": request.class_id}
        elif request.annotation_task == "pose_estimation":
            data_dict = {"pcl": np_cloud, "pose": pose_mask}
        else:
            rospy.logerr("annotation task is 1: Acc, 2: segmentation, 3: cllustering, 4: pose_estimation")
        hdf5_function.write_hdf5(self.hdf5_object, data_dict, index)
        response = RecordServiceResponse()
        response.ok = True
        return response
        
        
    def set_parameter(self):
        param_list = rosparam.get_param(rospy.get_name() + "/record_server/")
        self.hdf5_record_service_name = param_list["hdf5_record_service_name"]
        self.hdf5_file_dir = param_list["record_hdf5_file_dir"]
        self.hdf5_file_name = param_list["record_hdf5_file_name"]
        self.hdf5_save_interval = param_list["hdf5_save_interval"]
        

if __name__=='__main__':
    RecordServiceClass()
    rospy.spin()
    
