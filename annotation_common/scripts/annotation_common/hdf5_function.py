#!/usr/bin/env python3
import os
import h5py


def concatenate_hdf5(dir_path, out_file_name):
    files = os.listdir(dir_path)
    keys_1_all = []
    keys_2 = []
    all_data = []
    for i in range(len(files)):
        with h5py.File(os.path.join(dir_path, files[i]), mode="r") as f:
            if i == 0:
                keys_1 = []
                for k1 in f.keys():
                    keys_1.append(k1)
                for k2 in f[keys_1[0]].keys():
                    keys_2.append(k2)
                keys_1_all.append(keys_1)
            else:
                keys_1 = []
                for k1 in f.keys():
                    keys_1.append(k1)
                keys_1_all.append(keys_1)
    for i in range(len(files)):
        keys_1 = []
        with h5py.File(os.path.join(dir_path, files[i]), mode="r") as f:
            for j in range(len(f.keys())):
                part_data = []
                for k in range(len(keys_2)):
                    part_data.append(f[keys_1_all[i][j]][keys_2[k]][()])
                all_data.append(part_data)
    with h5py.File(os.path.join(dir_path, out_file_name), mode="w") as f:
        for i in range(len(all_data)):
            f.create_group('data_' + str(i+1))
            for j in range(len(keys_2)):
                f['data_' + str(i+1)].create_dataset(keys_2[j], data=all_data[i][j], compression="lzf")
    for ff in files:
        os.remove(os.path.join(dir_path, ff))


def write_hdf5(h5pyObject, data_dict, index):
    data_group = h5pyObject.create_group("data_" + str(index))
    for key, value in data_dict.items():
        data_group.create_dataset(key, data=value, compression="lzf")
    h5pyObject.flush()

def open_writed_hdf5(file_path):
    return h5py.File(file_path, "w")

def close_hdf5(h5pyObject):
    h5pyObject.flush()
    h5pyObject.close()