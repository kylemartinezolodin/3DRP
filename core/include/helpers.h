#ifndef __HELPERS_H__s
#define __HELPERS_H__

// std
#include <string>

// qt
//#include <QTime>
//#include <QString>

// vtk
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>

// pcl
#include <pcl/PolygonMesh.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/io/vtk_io.h>
//#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>  // loadPolygonFileOBJ


namespace _3DRPHelpers {

    // IO
    bool pclLoadSTL(const std::string fileNamePath, pcl::PolygonMesh& polyMesh);
    bool pclSaveSTL(const std::string fileNamePath, pcl::PolygonMesh& polyMesh, bool isBinaryFormat);

    bool vtkLoadSTL(const std::string fileNamePath, vtkSmartPointer<vtkPolyData> polyData);
    bool vtkSaveSTL(const std::string fileNamePath, const vtkSmartPointer<vtkPolyData> polyData, bool isBinaryFormat);

    // bool pclPLYReader(const std::string fileNamePath, pcl::PointCloud<pcl::PointXYZRGBA> &cloud);

    std::string getFileExtension(const std::string fileNamePath);
    std::string getFileExtension(const char* fileNamePath);

    bool ReadPolyData(const char* fileName, vtkSmartPointer<vtkPolyData> polyData);
    vtkSmartPointer<vtkPolyData> ReadPolyData(const char* fileName);

    void offConvertToObj(std::string fileName);

    // DATATYPE CONVERSION
	pcl::PolygonMesh vtkModelToPCLModel(vtkSmartPointer<vtkPolyData> vtkModel);
	vtkSmartPointer<vtkPolyData> pclModelToVTKModel(pcl::PolygonMesh pclModel);

    // remember there are usable PCL functions from "#include <pcl/io/vtk_lib_io.h>"
    //void pcl::io::pointCloudTovtkPolyData(const pcl::PCLPointCloud2Ptr &cloud, vtkSmartPointer<vtkPolyData>& poly_data);
    //void pcl::io::pointCloudTovtkPolyData(const pcl::PointCloud<PointT> &cloud,vtkPolyData* const polydata); 
    //void pcl::io::vtkPolyDataToPointCloud(vtkPolyData* const polydata, pcl::PointCloud<PointT>& cloud);


    // DATA MANIPULATION
    void setPointColor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int r, int g, int b);
}
#endif
