#ifndef RECONSTRUCTION
#define RECONSTRUCTION

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/io/ply_io.h>

#include <pcl/point_types.h>

#include <pcl/common/common.h>

#include <pcl/search/kdtree.h>

#include <pcl/PolygonMesh.h>

#include <pcl/filters/passthrough.h>

#include <pcl/features/normal_3d_omp.h>

#include <pcl/surface/poisson.h>

namespace _3DRPCore {
	namespace Reconstruction {
		std::string print_something();

		//pcl::PolygonMesh poisson(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud);
		pcl::PolygonMesh poisson(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
		//pcl::PolygonMesh poisson(pcl::PointCloud<pcl::PointNormal>::Ptr pointcloudWithNormals); // THIS FUNCTION WILL BE A CONTAINER WHERE AN ACTUAL RECONSTRUCTION HAPPENS
	}
}

#endif // !RECONSTRUCTION
