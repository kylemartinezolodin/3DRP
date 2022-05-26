#include <iostream>
#include "postprocessing.h"

#include <CGAL/Polygon_mesh_processing/corefinement.h>
namespace PMP = CGAL::Polygon_mesh_processing;

bool _3DRPCore::Postprocessing::booleanUnion(Polyhedron mesh1, Polyhedron mesh2, Polyhedron &out) {
    return PMP::corefine_and_compute_union(mesh1, mesh2, out);
}

#include <vector>
typedef Polyhedron::Vertex_handle                             Vertex_handle;
typedef Polyhedron::Halfedge_handle                           Halfedge_handle;
typedef Polyhedron::Facet_handle                              Facet_handle;

#include <CGAL/Polygon_mesh_processing/triangulate_hole.h> // for triangulate_refine_and_fair_hole()

int _3DRPCore::Postprocessing::holeFilling(Polyhedron &poly) {
    // Incrementally fill the holes
    unsigned int nb_holes = 0;
    for (Halfedge_handle h : halfedges(poly))
    {
        if (h->is_border())
        {
            std::vector<Facet_handle>  patch_facets;
            std::vector<Vertex_handle> patch_vertices;
            bool success = std::get<0>(PMP::triangulate_refine_and_fair_hole(poly,
                h,
                std::back_inserter(patch_facets),
                std::back_inserter(patch_vertices),
                PMP::parameters::vertex_point_map(get(CGAL::vertex_point, poly))
                .geom_traits(Kernel())));
            std::cout << " Number of facets in constructed patch: " << patch_facets.size() << std::endl;
            std::cout << " Number of vertices in constructed patch: " << patch_vertices.size() << std::endl;
            std::cout << " Fairing : " << (success ? "succeeded" : "failed") << std::endl;
            ++nb_holes;
        }
    }
    return nb_holes;
}