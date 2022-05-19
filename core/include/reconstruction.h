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


// types
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point;
//typedef Kernel::Vector_3 Vector;


#include <CGAL/Polyhedron_3.h>
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;


// Point with normal vector stored as a std::pair.
//typedef std::pair<Point, Vector> Pwn;

#include <CGAL/Point_set_3.h>
typedef CGAL::Point_set_3<Point> Point_set;

namespace _3DRPCore {
	namespace Reconstruction {
		std::string print_something();

		//pcl::PolygonMesh poisson(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud);
		pcl::PolygonMesh poisson(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
		//pcl::PolygonMesh poisson(pcl::PointCloud<pcl::PointNormal>::Ptr pointcloudWithNormals); // THIS FUNCTION WILL BE A CONTAINER WHERE AN ACTUAL RECONSTRUCTION HAPPENS
		
		void scaleSpace(Point_set points);
	}
}

#endif // !RECONSTRUCTION
