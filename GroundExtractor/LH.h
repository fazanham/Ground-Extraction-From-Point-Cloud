#pragma once

/*
* LH.h
* Opensource libraries used:
* Point Cloud Library (PCL v1.8.0)
* OpenCV (v3.1)
* Fade2D (v1.73)
* Created on: September, 2017
* Author: Faizaan Naveed
*/

#ifndef LH_H
#define LH_H

#include <fstream>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <bitset>
#include <vector>
#include <map>
#include <algorithm>
#include <list>
#include <windows.h>
#include <iostream>
#include <tuple>

#include <opencv2\core\core.hpp>
#include <opencv2\imgproc.hpp>
#include <opencv\cv.hpp>
#include <opencv2\core\mat.hpp>
#include <opencv2\opencv.hpp>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ascii_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/PolygonMesh.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>

#include <include_fade25d/Fade_2D.h>

#endif