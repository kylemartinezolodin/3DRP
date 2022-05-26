#include <iostream>
#include "reconstruction.h"


using namespace std;
//using namespace pcl;

std::string _3DRPCore::Reconstruction::print_something() {
	return "reconstructing..";
}

//pcl::PolygonMesh poisson(pcl::PointCloud<pcl::PointNormal>::Ptr pointcloudWithNormals) {
//	pcl::PolygonMesh a;
//	return a;
//}
 
pcl::PolygonMesh _3DRPCore::Reconstruction::poisson(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

    // passthrough filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PassThrough<pcl::PointXYZ> filter;
    filter.setInputCloud(cloud);
    filter.filter(*filtered);

    // estimate normal
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNumberOfThreads(8);
    ne.setInputCloud(filtered);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);

    ne.setRadiusSearch(0.01);
    //ne.setRadiusSearch(0.1);
    Eigen::Vector4f centroid;
    compute3DCentroid(*filtered, centroid);
    ne.setViewPoint(centroid[0], centroid[1], centroid[2]);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>());
    ne.compute(*cloud_normals);

    // flip normal
    for (size_t i = 0; i < cloud_normals->size(); ++i) {
        cloud_normals->points[i].normal_x *= -1;
        cloud_normals->points[i].normal_y *= -1;
        cloud_normals->points[i].normal_z *= -1;
    }

    // combine cloud point and estimated normal
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smoothed_normals(new pcl::PointCloud<pcl::PointNormal>());
    pcl::concatenateFields(*filtered, *cloud_normals, *cloud_smoothed_normals);

    // cloud w/ normals poisson reconstruction into mesh
    pcl::Poisson<pcl::PointNormal> poisson;
    poisson.setDepth(9);
    poisson.setInputCloud(cloud_smoothed_normals);
    pcl::PolygonMesh mesh;
    poisson.reconstruct(mesh);

    return mesh;
}


#include <CGAL/Scale_space_surface_reconstruction_3.h>
#include <CGAL/Scale_space_reconstruction_3/Advancing_front_mesher.h>
#include <CGAL/Scale_space_reconstruction_3/Jet_smoother.h>

typedef CGAL::Scale_space_surface_reconstruction_3<Kernel>                    Reconstructor;
typedef CGAL::Scale_space_reconstruction_3::Advancing_front_mesher<Kernel>    Mesher;
typedef CGAL::Scale_space_reconstruction_3::Jet_smoother<Kernel>              Smoother;

typedef Reconstructor::Facet_const_iterator                   Facet_iterator;

#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h> // for CGAL::Polygon_mesh_processing::IO::read_polygon_mesh()
#include <CGAL/Polygon_mesh_processing/orientation.h> // for Polygon_mesh_processing::orient()

Polyhedron _3DRPCore::Reconstruction::scaleSpace(Point_set points) {
    // Construct the mesh in a scale space.
    Reconstructor reconstruct(points.points().begin(), points.points().end());
    reconstruct.increase_scale<Smoother>(4);
    //reconstruct.reconstruct_surface(Mesher(0.5)); // set to low when imput has low vertex count
     reconstruct.reconstruct_surface(Mesher(3));  // set to high when imput has high vertex count; bunny70k

    std::ofstream out("temp.off");
    out << "OFF" << std::endl << points.size() << " " << reconstruct.number_of_facets() << " 0" << std::endl;

    for (Point_set::iterator it = points.begin(); it != points.end(); ++it)
        out << points.point(*it) << std::endl;

    for (Reconstructor::Facet_iterator it = reconstruct.facets_begin();
        it != reconstruct.facets_end(); ++it)
        out << "3 " << (*it)[0] << " " << (*it)[1] << " " << (*it)[2] << std::endl;

    Polyhedron outputMesh;
    if (CGAL::Polygon_mesh_processing::IO::read_polygon_mesh("temp.off", outputMesh)) {
        CGAL::Polygon_mesh_processing::orient(outputMesh); // assures that reconstructed mesh is oriented outward , since CGAL's scale space tends to return an inverted mesh
        std::cerr << "Scale Space Done." << std::endl;
    }
    else
        std::cerr << "Scale Space Error ." << std::endl;

    return outputMesh;
}


#include <array>
#include <CGAL/Advancing_front_surface_reconstruction.h>
#include <CGAL/Surface_mesh.h>

typedef std::array<std::size_t, 3> Facet;

typedef Kernel::Point_3  Point_3;

typedef CGAL::Surface_mesh<Point_3> Mesh;


struct Construct {
    Mesh& mesh;

    template < typename PointIterator>
    Construct(Mesh& mesh, PointIterator b, PointIterator e)
        : mesh(mesh)
    {
        for (; b != e; ++b) {
            boost::graph_traits<Mesh>::vertex_descriptor v;
            v = add_vertex(mesh);
            mesh.point(v) = *b;
        }
    }

    Construct& operator=(const Facet f)
    {
        typedef boost::graph_traits<Mesh>::vertex_descriptor vertex_descriptor;
        typedef boost::graph_traits<Mesh>::vertices_size_type size_type;
        mesh.add_face(vertex_descriptor(static_cast<size_type>(f[0])),
            vertex_descriptor(static_cast<size_type>(f[1])),
            vertex_descriptor(static_cast<size_type>(f[2])));
        return *this;
    }

    Construct&
        operator*() { return *this; }

    Construct&
        operator++() { return *this; }

    Construct
        operator++(int) { return *this; }

};


#include <CGAL/boost/graph/copy_face_graph.h>

Polyhedron _3DRPCore::Reconstruction::advancingFaceFront(std::vector<Point> points) {
    Mesh m;

    Construct construct(m, points.begin(), points.end());

    CGAL::advancing_front_surface_reconstruction(points.begin(),
        points.end(),
        construct);

    Polyhedron output_mesh;
    CGAL::copy_face_graph(m, output_mesh);
    return output_mesh;
}