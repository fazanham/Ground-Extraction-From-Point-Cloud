#pragma once

/*
* TerrainFilter.h
* Class for removing non-ground points from the LiDAR point cloud using triangulation. 
* Created on: September, 2017
* Author: Faizaan Naveed
*/

#ifndef TERRAINFILTER_H
#define TERRAINFILTER_H

#include "BlockProcessor.h"
#include "LH.h"

#define PI 3.14159265358979323846 // custom pi ;)

class CTerrainFilter
{
public:
	CTerrainFilter();
	CTerrainFilter(const char* input_file_name, double grid_size, int disk_radius, double height_difference);

	// ATTRIBUTES
	
	double size_grid; // The size of the grid for interpolation
	int Disk_Radius; // The size of the disk structuring element to remove the non-ground points via morphological operation
	double Height_Difference_Threshold; // Height difference threshold for removing non-ground points from a coarse approximation of the ground (DEM)
	// METHODS

	/* Use improved progressive TIN densification to refine the ground points estimates */
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> Triangulation(int Num_points_for_computing_mean, double Num_of_std_dev_for_removing_outliers, double Angle_threshold, double Distance_threshold);

	/* Write the ground points to a PCD ascii file
	@param outfiledir: directory of the outfile
	@param cloud: the cloud containing densified ground points from Triangulation
	*/
	void writeData2ascii(char *outfiledir, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud);

private:

	// ATTRIBUTES 

	CBlockProcessor * Block_Process_Obj; // Private object for block processing the las file

	// METHODS

	/* Generates a convex hull for the given point cloud
	@param mat: the input point cloud
	@param mat_f_count: total number of points in the cloud
	@return a PCL point cloud defining the convex hull
	*/
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> ConvexHull(double ** mat, int mat_f_count);

	/*Perform a morphological opening operation to coarsely remove the non-ground points from the cloud.*/
	std::vector<cv::Point3f> OpenOperation();

	/* Generates a disk structuring element for removing non-ground points
	@param Radius: The radius of the disk
	@return A disk structuring element with radius Radius
	*/
	cv::Mat strelDisk(int Radius);

};

#endif