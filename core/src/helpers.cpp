#include "helpers.h"

#include <vtksys/SystemTools.hxx>
#include <vtkPolyDataReader.h>

// meshes
#include <vtkSphereSource.h>
#include <vtkOBJReader.h>
#include <vtkPLYReader.h>
#include <vtkPolyDataReader.h>
#include <vtkSTLReader.h>

// points
#include <vtkParticleReader.h>
#include <vtkSimplePointsReader.h>

// for offConvertToObj()
#include <CGAL/boost/graph/IO/polygon_mesh_io.h> 

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;


bool _3DRPHelpers::pclLoadSTL(const std::string fileNamePath, pcl::PolygonMesh& polyMesh) {
	pcl::io::loadPolygonFileSTL(fileNamePath, polyMesh);

	return polyMesh.polygons.size() != 0;
}
bool _3DRPHelpers::pclSaveSTL(const std::string fileNamePath, pcl::PolygonMesh& polyMesh, bool isBinaryFormat) {
	if (polyMesh.polygons.size() == 0) return false;
	return pcl::io::savePolygonFileSTL(fileNamePath, polyMesh, isBinaryFormat);
}

bool _3DRPHelpers::vtkLoadSTL(const std::string fileNamePath, vtkSmartPointer<vtkPolyData> polyData) {

	vtkNew<vtkSTLReader> reader;
	reader->SetFileName(fileNamePath.c_str());
	reader->Update();
	polyData = reader->GetOutput();

	return polyData->GetNumberOfPoints() == 0;
}
bool _3DRPHelpers::vtkSaveSTL(const std::string fileNamePath, const vtkSmartPointer<vtkPolyData> polyData, bool isBinaryFormat) {

	if (polyData->GetNumberOfPoints() == 0) return false;

	// FROM: https://kitware.github.io/vtk-examples/site/Cxx/IO/WriteSTL/
	vtkNew<vtkSTLWriter> stlWriter;
	stlWriter->SetFileName(fileNamePath.c_str());
	stlWriter->SetFileType(isBinaryFormat);
	stlWriter->SetInputData(polyData);
	stlWriter->Write();

	return true;
}

std::string _3DRPHelpers::getFileExtension(const std::string fileNamePath) {
    std::string extension =
        vtksys::SystemTools::GetFilenameLastExtension(fileNamePath);

    // Drop the case of the extension
    std::transform(extension.begin(), extension.end(), extension.begin(),
        ::tolower);

    return extension;
}

std::string _3DRPHelpers::getFileExtension(const char* fileNamePath) {
    std::string extension =
        vtksys::SystemTools::GetFilenameLastExtension(std::string(fileNamePath));

    // Drop the case of the extension
    std::transform(extension.begin(), extension.end(), extension.begin(),
        ::tolower);

    return extension;
}

// currently broken  please use the other override (idk y i cannot mutate the passed polydata parameter)
bool _3DRPHelpers::ReadPolyData(const char* fileName, vtkSmartPointer<vtkPolyData> polyData)
{
    std::string extension = _3DRPHelpers::getFileExtension(fileName);

    if (extension == ".ply" && false) // Explicitly disabled. Does not work on some test .ply file
    {
        vtkNew<vtkPLYReader> reader;
        reader->SetFileName(fileName);
        reader->Update();
        polyData.TakeReference(reader->GetOutput());
    }
    else if (extension == ".obj")
    {
        vtkNew<vtkOBJReader> reader;
        reader->SetFileName(fileName);
        reader->Update();
        polyData.TakeReference(reader->GetOutput());
    }
    else if (extension == ".stl")
    {
        vtkNew<vtkSTLReader> reader;
        reader->SetFileName(fileName);
        reader->Update();
        polyData.TakeReference(reader->GetOutput());
    }
    else if (extension == ".vtk")
    {
        vtkNew<vtkPolyDataReader> reader;
        reader->SetFileName(fileName);
        reader->Update();
        polyData.TakeReference(reader->GetOutput());
    }
    //else if (extension == ".pcd")
    //{
    //        MyCloud myCloud;
    //        string filePath = fromQString(fileInfo.filePath());
    //        int status = -1;
    //        status = pcl::io::loadPCDFile(filePath, *(myCloud.cloud));
    //    
    //        bool hasCloud = (status == 0);
    //        // There is no polygon mesh loader for pcd file in PCL
    //        bool hasMesh = false;
    //        myCloud.init(fileInfo, hasCloud, hasMesh);
    //    
    //        return myCloud;
    //}
    else // try to decode file as a point cloud
    {
        // FROM: https://kitware.github.io/vtk-examples/site/Cxx/IO/ParticleReader/
        // Particles.raw supplied by VTK is big endian encoded
        // std::string filePath = "C:\\VTK\\vtkdata-5.8.0\\Data\\Particles.raw";
        vtkNew<vtkParticleReader> raw_points;

        raw_points->SetFileName(fileName);
        // if nothing gets displayed or totally wrong, swap the endianness
        raw_points->SetDataByteOrderToBigEndian();
        raw_points->Update();
        polyData.TakeReference(raw_points->GetOutput());

        if (polyData->GetNumberOfPoints() == 0 || polyData->GetNumberOfVerts() == 0)
        {
            // FROM: https://kitware.github.io/vtk-examples/site/Cxx/IO/SimplePointsReader/
            vtkNew<vtkSimplePointsReader> simple_points;
            simple_points->SetFileName(fileName);
            simple_points->Update();
            polyData = simple_points->GetOutput();
        }


    }

    if (polyData->GetNumberOfPoints() == 0 || polyData->GetNumberOfVerts() == 0) // vertices is the most atomic geametric-data when ther is no verts, there is no geometry
        return false;

    return true;
}


vtkSmartPointer<vtkPolyData> _3DRPHelpers::ReadPolyData(const char* fileName)
{
    vtkSmartPointer<vtkPolyData> polyData;

    std::string extension = _3DRPHelpers::getFileExtension(fileName);

    if (extension == ".ply")
    {
        vtkNew<vtkPLYReader> reader;
        reader->SetFileName(fileName);
        reader->Update();
        polyData = reader->GetOutput();
    }
    else if (extension == ".obj")
    {
        vtkNew<vtkOBJReader> reader;
        reader->SetFileName(fileName);
        reader->Update();
        polyData = reader->GetOutput();
    }
    else if (extension == ".stl")
    {
        vtkNew<vtkSTLReader> reader;
        reader->SetFileName(fileName);
        reader->Update();
        polyData = reader->GetOutput();
    }
    else if (extension == ".vtk")
    {
        vtkNew<vtkPolyDataReader> reader;
        reader->SetFileName(fileName);
        reader->Update();
        polyData = reader->GetOutput();
    }
    else // try to decode file as a point cloud
    {
        // FROM: https://kitware.github.io/vtk-examples/site/Cxx/IO/ParticleReader/
        // Particles.raw supplied by VTK is big endian encoded
        // std::string filePath = "C:\\VTK\\vtkdata-5.8.0\\Data\\Particles.raw";
        vtkNew<vtkParticleReader> raw_points;

        raw_points->SetFileName(fileName);
        // if nothing gets displayed or totally wrong, swap the endianness
        raw_points->SetDataByteOrderToBigEndian();
        raw_points->Update();
        polyData = raw_points->GetOutput();

        if (polyData->GetNumberOfPoints() == 0 || polyData->GetNumberOfVerts() == 0)
        {
            // FROM: https://kitware.github.io/vtk-examples/site/Cxx/IO/SimplePointsReader/
            vtkNew<vtkSimplePointsReader> simple_points;
            simple_points->SetFileName(fileName);
            simple_points->Update();
            polyData = simple_points->GetOutput();
        }


    }

    return polyData;
}

pcl::PolygonMesh _3DRPHelpers::vtkModelToPCLModel(vtkSmartPointer<vtkPolyData> vtkModel) {
	pcl::PolygonMesh polyMesh;

	vtkSaveSTL("temp.stl", vtkModel, true);
	pclLoadSTL("temp.stl", polyMesh);
	std::string s = "temp.stl";
	remove(s.c_str());

	return polyMesh;
}

vtkSmartPointer<vtkPolyData> _3DRPHelpers::pclModelToVTKModel(pcl::PolygonMesh pclModel) {
	vtkSmartPointer<vtkPolyData> polyData;

	pclSaveSTL("temp.stl", pclModel, true);
	vtkLoadSTL("temp.stl", polyData);
	std::string s = "temp.stl";
	remove(s.c_str());

	return polyData;
}

void _3DRPHelpers::setPointColor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int r, int g, int b) {
    for (int i = 0; i != cloud->points.size(); ++i) {
        cloud->points[i].r = r;
        cloud->points[i].g = g;
        cloud->points[i].b = b;
    }
}

void _3DRPHelpers::offConvertToObj(std::string offFileName) {
    Polyhedron polygon;
    if (!CGAL::IO::read_polygon_mesh(offFileName, polygon))
       std::cerr << "offConvertToObj(): Unable to read file" << std::endl;

    offFileName.resize(offFileName.find_last_of('.'));

    if (!CGAL::IO::write_polygon_mesh(offFileName +".obj", polygon))
       std::cerr << "offConvertToObj(): Unable to write file" << std::endl;
}