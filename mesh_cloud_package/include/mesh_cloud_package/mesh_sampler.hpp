#pragma once
#include <pcl/io/vtk_lib_io.h>
#include <vtkVersion.h>
#include <vtkOBJReader.h>
#include <vtkPLYReader.h>
#include <vtkPolyDataMapper.h>
#include <vtkTriangle.h>
#include <vtkTriangleFilter.h>
#include <iostream>
#include <util/util_msg_data.hpp>


double uniform_deviate(int);

void randomPointTriangle(float a1, float a2, float a3,
                                float b1, float b2, float b3,
                                float c1, float c2, float c3, Eigen::Vector4f &p);


void randPSurface(vtkPolyData *polydata, std::vector<double> *cumulativeAreas, double totalArea,
                            Eigen::Vector4f &p);


void uniform_sampling(vtkSmartPointer<vtkPolyData> polydata, size_t n_samples,
                        pcl::PointCloud<PclXyz> &cloud_out);
