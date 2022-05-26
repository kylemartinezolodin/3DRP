#ifndef POSTPROCESSING
#define POSTPROCESSING


// types
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point;
//typedef Kernel::Vector_3 Vector;


#include <CGAL/Polyhedron_3.h>
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;

namespace _3DRPCore {
	namespace Postprocessing {
		bool booleanUnion(Polyhedron mesh1, Polyhedron mesh2, Polyhedron &out);
		int holeFilling(Polyhedron &poly);
	}
}

#endif // POSTPROCESSING