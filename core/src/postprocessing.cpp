#include <iostream>
#include "postprocessing.h"
#include <CGAL/Nef_polyhedron_3.h>
#include <CGAL/Exact_integer.h>
#include <CGAL/Homogeneous.h>
#include <CGAL/Nef_polyhedron_3.h>
#include <CGAL/Nef_nary_union_3.h>
	

	typedef CGAL::Homogeneous<CGAL::Exact_integer>  Kernel;
	typedef CGAL::Nef_polyhedron_3<Kernel> Nef_polyhedron;


std::string _3DRPCore::Postprocessing::print_something() {
	

	

	/*
	for (int i = 0; i != filePathList.size(); i++) {
		timeStart(); // time start
		mycloud.cloud.reset(new PointCloudT); // Reset cloud
		QFileInfo fileInfo(filePathList[i]);
		std::string filePath = fromQString(fileInfo.filePath());
		std::string fileName = fromQString(fileInfo.fileName());
		
		// begin loading
		ui.statusBar->showMessage(
			fileInfo.fileName() + ": " + QString::number(i) + "/" + QString::number(filePathList.size())
			+ " point cloud loading..."
		);

		mycloud = fileIO.load(fileInfo);
		if (!mycloud.isValid) {
			// TODO: deal with the error, print error info in console?
			debug("invalid cloud.");
			continue;
		}
		mycloud.viewer = viewer;
		mycloud_vec.push_back(mycloud);

		timeCostSecond = timeOff(); // time off

		consoleLog(
			"Open",
			toQString(mycloud.fileName),
			toQString(mycloud.filePath),
			"Time cost: " + timeCostSecond + " s, Points: " + QString::number(mycloud.cloud->points.size())
		);

		// update tree widget
		QTreeWidgetItem *cloudName = new QTreeWidgetItem(QStringList()
			<< toQString(mycloud.fileName));
		cloudName->setIcon(0, QIcon(":/Resources/images/icon.png"));
		ui.dataTree->addTopLevelItem(cloudName);

		total_points += mycloud.cloud->points.size();
		
	}
	ui.statusBar->showMessage("");
	showPointcloudAdd();
	setPropertyTable();
	
	*/
	//Convert nef3 to mesh then save mesh method
	
	/*

	typedef CGAL::Exact_predicates_exact_constructions_kernel Exact_kernel;
	typedef CGAL::Polyhedron_3<Exact_kernel> Polyhedron;
	typedef CGAL::Surface_mesh<Exact_kernel::Point_3> Surface_mesh;
	typedef CGAL::Nef_polyhedron_3<Exact_kernel> Nef_polyhedron;

	Polyhedron cube1, cube2;
	fill_cube_1(cube1);
	fill_cube_2(cube2);
	Nef_polyhedron nef1(cube1);
	Nef_polyhedron nef2(cube2);
	// compute the difference of the nested cubes
	Nef_polyhedron nef = nef1 - nef2;
	// output the result into a Surface_mesh
	Surface_mesh output;
	CGAL::convert_nef_polyhedron_to_polygon_mesh(nef, output);
	std::ofstream out;
	out.open("out.off");
	out << output;
	out.close();
	*/

	//Union of 2 volumes
	

	return "postprocessing..";
}
