#include <mesh_cloud_package/mesh_sampler.hpp>

inline double uniform_deviate(int seed)
{
    double ran = seed * (1.0 / (RAND_MAX + 1.0));
    return ran;
}

inline void randomPointTriangle(float a1, float a2, float a3,
                                float b1, float b2, float b3,
                                float c1, float c2, float c3, Eigen::Vector4f &p)
{
    float r1 = static_cast<float>(uniform_deviate(rand()));
    float r2 = static_cast<float>(uniform_deviate(rand()));
    float rlsqr = sqrtf(r1);
    float OneMinR1Sqr = (1 - rlsqr);
    float OneMinR2 = (1 - r2);
    a1 *= OneMinR1Sqr;
    a2 *= OneMinR1Sqr;
    a3 *= OneMinR1Sqr;
    b1 *= OneMinR2;
    b2 *= OneMinR2;
    b3 *= OneMinR2;
    c1 = rlsqr * (r2 * c1 + b1) + a1;
    c2 = rlsqr * (r2 * c2 + b2) + a2;
    c3 = rlsqr * (r2 * c3 + b3) + a3;
    p[0] = c1;
    p[1] = c2;
    p[2] = c3;
    p[3] = 0;
}

inline void randPSurface(vtkPolyData *polydata, std::vector<double> *cumulativeAreas, double totalArea,
                            Eigen::Vector4f &p)
{
    // Util::message_show("36", "ok");
    float r = static_cast<float>(uniform_deviate(rand()) * totalArea);
    // Util::message_show("38", "ok");
    std::vector<double>::iterator low = std::lower_bound(cumulativeAreas->begin(), cumulativeAreas->end(), r);
    // Util::message_show("40", "ok");
    vtkIdType el = vtkIdType(low - cumulativeAreas->begin());
    // Util::message_show("42", "ok");
    double A[3], B[3], C[3];
    vtkIdType npts = 0;
    vtkIdType *ptIds = NULL;
    // Util::message_show("46", "ok");
    polydata->GetCellPoints(el, npts, ptIds);
    // Util::message_show("48", "ok");
    polydata->GetPoint(ptIds[0], A);
    // Util::message_show("50", "ok");
    polydata->GetPoint(ptIds[1], B);
    // Util::message_show("52", "ok");
    polydata->GetPoint(ptIds[2], C);
    // Util::message_show("54", "ok");
    randomPointTriangle(float(A[0]), float(A[1]), float(A[2]), float(B[0]), float(B[1]),
                        float(B[2]), float(C[0]), float(C[1]), float(C[2]), p);
}

void uniform_sampling(vtkSmartPointer<vtkPolyData> polydata, size_t n_samples,
                        pcl::PointCloud<PclXyz> &cloud_out)
{
    // Util::message_show("build_cells, cloud_size", cloud_out.points.size());
    polydata->BuildCells();
    // Util::message_show("GetPolys", "ok");
    vtkSmartPointer<vtkCellArray> cells = polydata->GetPolys();
    double p1[3], p2[3], p3[3], totalArea = 0;
    // Util::message_show("cumulativeAreas", "ok");
    std::vector<double> cumulativeAreas(cells->GetNumberOfCells(), 0);
    size_t i = 0;
    vtkIdType npts = 0, *ptIds = NULL;
    // Util::message_show("for_loop", "ok");
    for (cells->InitTraversal(); cells->GetNextCell(npts, ptIds); i++)
    {
        polydata->GetPoint(ptIds[0], p1);
        polydata->GetPoint(ptIds[1], p2);
        polydata->GetPoint(ptIds[2], p3);
        totalArea += vtkTriangle::TriangleArea(p1, p2, p3);
        cumulativeAreas[i] = totalArea;
    }
    // Util::message_show("cloud_out", "ok");
    cloud_out.points.resize(n_samples);
    cloud_out.width = static_cast<std::uint32_t>(n_samples);
    cloud_out.height = 1;
    // Util::message_show("rand_Psurface", "ok");
    for (i = 0; i < n_samples; i++)
    {
        Eigen::Vector4f p;
        randPSurface(polydata, &cumulativeAreas, totalArea, p);
        cloud_out.points[i].x = p[0];
        cloud_out.points[i].y = p[1];
        cloud_out.points[i].z = p[2];
    }
}
