#include "TerrainFilter.h"

CTerrainFilter::CTerrainFilter() :
	size_grid(0),
	Disk_Radius(0),
	Height_Difference_Threshold(0)
{
	Block_Process_Obj = new CBlockProcessor();
}

CTerrainFilter::CTerrainFilter(const char * input_file_name, double grid_size, int disk_radius, double height_difference) :
	size_grid(grid_size),
	Disk_Radius(disk_radius),
	Height_Difference_Threshold(height_difference)
{
	Block_Process_Obj = new CBlockProcessor(input_file_name, grid_size);
}

boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> CTerrainFilter::ConvexHull(double ** mat, int mat_f_count)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr c_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	for (size_t i = 0; i < mat_f_count; i++) 
		c_cloud->push_back(pcl::PointXYZ(mat[i][0], mat[i][1], 1.));
	
	pcl::ConvexHull<pcl::PointXYZ> cHull;
	pcl::PointCloud<pcl::PointXYZ> cHull_Points;
	std::vector<pcl::Vertices> polygons;
	cHull.setInputCloud(c_cloud);
	cHull.reconstruct(cHull_Points, polygons);

	pcl::PointIndices hull_point_indicies;
	cHull.getHullPointIndices(hull_point_indicies);

	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> CHULL = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>);

	for (int i = 0; i < hull_point_indicies.indices.size(); i++) 
		CHULL->push_back(pcl::PointXYZ(mat[hull_point_indicies.indices[i]][0], mat[hull_point_indicies.indices[i]][1], 1.));
	
	return CHULL;
}

/*
* The function performs a morphological opening operation to determine potential
* ground seed points for triangulation. 
* Refer to Axelsson 2000, DEM generation from laser scanned data using adaptive TIN
*/
std::vector<cv::Point3f> CTerrainFilter::OpenOperation()
{

	double ** mat_f;
	int mat_f_count;

	std::tie(mat_f, mat_f_count) = Block_Process_Obj->BlockProcess();

	/*Generate convex hull*/
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> CHULL = CTerrainFilter::ConvexHull(mat_f, mat_f_count);

	// Caculate the numbers of the rows and columns to initialize the raster:
	int nx = ceil((Block_Process_Obj->B_Maximal_Data_Extent[0] - Block_Process_Obj->B_Maximal_Data_Extent[1]) / size_grid);
	if ((size_grid*nx - (Block_Process_Obj->B_Maximal_Data_Extent[0] - Block_Process_Obj->B_Maximal_Data_Extent[1])) < size_grid / 2)
		nx += 1;

	int ny = ceil((Block_Process_Obj->B_Maximal_Data_Extent[2] - Block_Process_Obj->B_Maximal_Data_Extent[3]) / size_grid) + 1;
	if ((size_grid*ny - (Block_Process_Obj->B_Maximal_Data_Extent[2] - Block_Process_Obj->B_Maximal_Data_Extent[3])) < size_grid / 2)
		ny += 1;

	/*
	* Find the minimum point within each grid.
	* The lowest point in each grid is selected to be rasterized.
	*/
	pcl::PointXYZ ** lowraster = (pcl::PointXYZ **)malloc(sizeof(double)*(nx + 1));
	for (int i = 0; i < nx + 1; i++) {
		lowraster[i] = (pcl::PointXYZ *)malloc(3 * sizeof(double)*(ny + 1));
		for (int j = 0; j < ny + 1; j++) {
			lowraster[i][j].x = 0.0;
			lowraster[i][j].y = 0.0;
			lowraster[i][j].z = 0.0;
		}
	}

	/*the count of points in the raster*/
	int matc = 0;

	/*the number of rows and cols*/
	int max_temp_idy = 0;
	int max_temp_idx = 0;

	/*Raster to hold the data*/
	for (int j = 0; j < mat_f_count; j++) {

		if (pcl::isPointIn2DPolygon(pcl::PointXYZ(mat_f[j][0], mat_f[j][1], 1.), *CHULL)) {

			/*Determine the integer ids of the location of the pixel value*/
			int temp_idy = floor(((1 / size_grid) * (mat_f[j][1] - Block_Process_Obj->B_Maximal_Data_Extent[3])));
			int temp_idx = floor(((1 / size_grid) * (mat_f[j][0] - Block_Process_Obj->B_Maximal_Data_Extent[1])));

			/*Keep count of the max row and col*/
			if (max_temp_idx < temp_idx)
				max_temp_idx = temp_idx;

			if (max_temp_idy < temp_idy)
				max_temp_idy = temp_idy;

			/*The grid is empty hence insert the value*/
			if ((lowraster[temp_idx][temp_idy].x == 0.0) && (lowraster[temp_idx][temp_idy].y == 0.0)) {
				lowraster[temp_idx][temp_idy].x = mat_f[j][0];
				lowraster[temp_idx][temp_idy].y = mat_f[j][1];
				lowraster[temp_idx][temp_idy].z = mat_f[j][2];

				matc++;
			}
			/*If the grid already has a value than check to see if its minimum*/
			else {
				if (lowraster[temp_idx][temp_idy].z > mat_f[j][2]) {
					lowraster[temp_idx][temp_idy].z = mat_f[j][2];
					matc++;
				}
			}
		}
	}

	/*Free the memory for mat_f*/
	for (int i = 0; i < mat_f_count; i++) {
		free(mat_f[i]);
	}
	free(mat_f);
	mat_f = NULL;

	/*Perform nearest neighbor interpolation to fill the holes*/
	for (int i = 0; i < max_temp_idx + 1; i++) {
		for (int j = 0; j < max_temp_idy + 1; j++) {
			if (lowraster[i][j].x == 0.0 && lowraster[i][j].y == 0.0) {

				/*Nearest Neighbor search for deriving the row and column*/
				double x_nn = (i * size_grid) + Block_Process_Obj->B_Maximal_Data_Extent[1];
				double y_nn = (j * size_grid) + Block_Process_Obj->B_Maximal_Data_Extent[3];

				/*Check if the point is inside the convex hull*/
				if (pcl::isPointIn2DPolygon(pcl::PointXYZ(x_nn, y_nn, 1.), *CHULL)) {

					double z_nn = 0;
					double dis_nn = std::numeric_limits<double>::max();

					/*Check the neighbors*/
					if (i > 0 && j > 0 && i < max_temp_idx + 1 && j < max_temp_idy + 1) {
						/*Search the 8 nearest neighbors*/
						for (int i_in = i - 1; i_in < i + 2; i_in++) {
							for (int j_in = j - 1; j_in < j + 2; j_in++) {
								if (i_in != i && j_in != j && lowraster[i_in][j_in].x != 0.0 && lowraster[i_in][j_in].y != 0.0) {
									double dis = sqrt(pow(x_nn - lowraster[i_in][j_in].x, 2) + pow(y_nn - lowraster[i_in][j_in].y, 2));
									/*find the closest neighbor in terms of euclidean distance and assign its pixel value*/
									if (dis_nn > dis) {
										dis_nn = dis;
										z_nn = lowraster[i_in][j_in].z;
									}
								}
							}
						}
					}

					lowraster[i][j].x = x_nn;
					lowraster[i][j].y = y_nn;
					lowraster[i][j].z = z_nn;
				}
			}
		}
	}

	/*Store the data into an opencv Mat data structure*/
	cv::Mat rasterize(ny, nx, CV_32F, cvScalar(0.));

	for (int i = 0; i < nx; i++) {
		for (int j = 0; j < ny; j++) {
			if (lowraster[i][j].x != 0.0 && lowraster[i][j].z != 0.0) {
				rasterize.at<float>(j, i) = lowraster[i][j].z;
			}
		}
	}

	/*
	Apply the opening operation to remove non-ground elements.
	A disk structuring element is optimal for this operation.
	*/
	cv::Mat dst;
	cv::Mat channel_blur;
	cv::Mat strl = strelDisk(Disk_Radius);

	/*Blurring and Opening operations*/
	cv::morphologyEx(rasterize, dst, cv::MORPH_OPEN, strl);
	cv::GaussianBlur(rasterize, channel_blur, cv::Size(9, 9), 2.95);

	/*Alienate the ground elements from the non ground elements*/
	int seed_points = 0;
	std::vector<cv::Point3f> vec_f;
	cv::Mat G_potential(ny, nx, CV_32F, cvScalar(0.));

	for (int i = 0; i < ny; i++) {
		for (int j = 0; j < nx; j++) {
			if (abs(channel_blur.at<float>(i, j) - dst.at<float>(i, j)) < Height_Difference_Threshold) {
				G_potential.at<float>(i, j) = rasterize.at<float>(i, j);
				if (lowraster[j][i].x != 0.0 && lowraster[j][i].z != 0.0) {
					vec_f.push_back(cv::Point3f(lowraster[j][i].x, lowraster[j][i].y, lowraster[j][i].z));
					seed_points++;
				}
			}
		}
	}

	return vec_f;
}

boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> CTerrainFilter::Triangulation(int Num_points_for_computing_mean, double Num_of_std_dev_for_removing_outliers, double Angle_threshold, double Distance_threshold)
{
	/* Read the las file */
	bool isLasRead = CTerrainFilter::Block_Process_Obj->LasReader();

	/* load the dataset in pcl::pointclouds */
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (isLasRead) {

		/* Check whether there are still points left to be processed */
		while (CTerrainFilter::Block_Process_Obj->IsPointCloudNotExhausted) {

			boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> original_cloud(new pcl::PointCloud<pcl::PointXYZ>);
			boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud(new pcl::PointCloud<pcl::PointXYZ>);

			std::vector<cv::Point3f> mat_f = CTerrainFilter::OpenOperation();
			int seed_points = mat_f.size();

			for (int i = 0; i < CTerrainFilter::Block_Process_Obj->Original_Cloud.size(); i++) {
				if (i < seed_points) {
					cloud->push_back(pcl::PointXYZ(mat_f[i].x, mat_f[i].y, mat_f[i].z));
				}
				original_cloud->push_back(pcl::PointXYZ(CTerrainFilter::Block_Process_Obj->Original_Cloud[i].x, CTerrainFilter::Block_Process_Obj->Original_Cloud[i].y, CTerrainFilter::Block_Process_Obj->Original_Cloud[i].z));
			}

			// Perform Statistical Outlier removal to remove remaining non-ground points
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

			std::vector<int> indices;
			pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

			pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
			sor.setInputCloud(cloud);
			sor.setMeanK(Num_points_for_computing_mean);
			sor.setStddevMulThresh(Num_of_std_dev_for_removing_outliers);
			sor.filter(*cloud_filtered);

			std::vector<GEOM_FADE25D::Point2> vInputPoints;

			for (int i = 0; i < cloud_filtered->size(); i++) {
				ground_cloud->push_back(pcl::PointXYZ(cloud_filtered->at(i).x, cloud_filtered->at(i).y, cloud_filtered->at(i).z));
				vInputPoints.push_back(GEOM_FADE25D::Point2(cloud_filtered->at(i).x, cloud_filtered->at(i).y, cloud_filtered->at(i).z));
			}

			/*Perform the initial triangulation*/
			GEOM_FADE25D::Fade_2D* pDt = new GEOM_FADE25D::Fade_2D;
			pDt->insert(vInputPoints);

			GEOM_FADE25D::Triangle2 *temp_TR;
			GEOM_FADE25D::Vector2 temp_V2;
			GEOM_FADE25D::Point2 temp_P2;

			for (pcl::PointCloud<pcl::PointXYZ>::iterator it = original_cloud->begin(); it != original_cloud->end(); it++) {
				temp_TR = pDt->locate(GEOM_FADE25D::Point2(it->x, it->y, it->z));
				if (temp_TR != NULL) {
					temp_V2 = temp_TR->getNormalVector();
					temp_P2 = temp_TR->getBarycenter();

					/*Point projected onto the plane of the triangle*/
					GEOM_FADE25D::Vector2 tempV(GEOM_FADE25D::Point2(it->x, it->y, it->z) - temp_P2);
					double dist = (tempV.x() * temp_V2.x()) + (tempV.y()*temp_V2.y()) + (tempV.z()*temp_V2.z());
					GEOM_FADE25D::Point2 proj_P = GEOM_FADE25D::Point2(it->x, it->y, it->z) - (dist * temp_V2);

					/*Compute the normal distance to the surface of the triangle*/
					double dis_N = sqrt(pow(proj_P.x() - it->x, 2) + pow(proj_P.y() - it->y, 2) + pow(proj_P.z() - it->z, 2));

					/*Compute the angles to the three vertices of the triangle*/
					double dis_P0 = sqrt(pow(it->x - temp_TR->getCorner(0)->x(), 2) + pow(it->y - temp_TR->getCorner(0)->y(), 2) + pow(it->z - temp_TR->getCorner(0)->z(), 2));
					double dis_P1 = sqrt(pow(it->x - temp_TR->getCorner(1)->x(), 2) + pow(it->y - temp_TR->getCorner(1)->y(), 2) + pow(it->z - temp_TR->getCorner(1)->z(), 2));
					double dis_P2 = sqrt(pow(it->x - temp_TR->getCorner(2)->x(), 2) + pow(it->y - temp_TR->getCorner(2)->y(), 2) + pow(it->z - temp_TR->getCorner(2)->z(), 2));

					/*Convert the angles to radiuans*/
					double ALPHA = asin(dis_N / dis_P0) * 180.0 / PI;
					double BETA = asin(dis_N / dis_P1) * 180.0 / PI;
					double GAMMA = asin(dis_N / dis_P2) * 180.0 / PI;

					/*Check against threshold to see if the point belongs in the triangle*/
					if ((ALPHA < Angle_threshold && BETA < Angle_threshold && GAMMA < Angle_threshold) && (dis_N < Distance_threshold)) {
						pDt->insert(GEOM_FADE25D::Point2(it->x, it->y, it->z));
						ground_cloud->push_back(pcl::PointXYZ(it->x, it->y, it->z));
					}
				}
			}
		}
		return ground_cloud;
	}
	else {
		printf("Fail to read las file\n");
		exit(-1);
	}
}

void CTerrainFilter::writeData2ascii(char * outfiledir, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud)
{

	int i = strlen(outfiledir);

	if (i > 0 && outfiledir != NULL) {

		FILE *ffp = fopen(outfiledir, "wt");

		for (pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud->begin(); it != cloud->end(); it++) {
			fprintf(ffp, "%f %f %f\n", it->x, it->y, it->z);
		}

		fflush(ffp);
		fclose(ffp);
	}
	else {
		printf("Invalid Filename\n");
		exit(-1);
	}
}

cv::Mat CTerrainFilter::strelDisk(int Radius)
{
	cv::Mat sel((2 * Radius - 1), (2 * Radius - 1), CV_8U, cv::Scalar(1));

	int borderWidth;

	//Missing cases: 1 and 2
	switch (Radius) {
	case 1:borderWidth = 1; break;
	case 2:borderWidth = 2; break;
	case 3: borderWidth = 0; break;
	case 4: borderWidth = 2; break;
	case 5: borderWidth = 2; break;
	case 6: borderWidth = 2; break;
	case 7: borderWidth = 2; break;
	case 8: borderWidth = 4; break;
	case 9: borderWidth = 4; break;
	case 10: borderWidth = 4; break;
	case 11: borderWidth = 6; break;
	case 12: borderWidth = 6; break;
	case 13: borderWidth = 6; break;
	case 14: borderWidth = 6; break;
	case 15: borderWidth = 8; break;
	case 16: borderWidth = 8; break;
	case 17: borderWidth = 8; break;
	case 18: borderWidth = 10; break;
	case 19: borderWidth = 10; break;
	case 20: borderWidth = 10; break;
	case 21: borderWidth = 10; break;
	case 22: borderWidth = 10; break;
	case 23: borderWidth = 12; break;
	case 24: borderWidth = 12; break;
	case 25: borderWidth = 14; break;
	case 26: borderWidth = 14; break;
	case 27: borderWidth = 14; break;
	case 28: borderWidth = 14; break;
	case 29: borderWidth = 16; break;
	case 30: borderWidth = 16; break;
	case 31: borderWidth = 16; break;
	case 32: borderWidth = 18; break;
	case 33: borderWidth = 18; break;

	}

	for (int i = 0; i < borderWidth; i++) {
		for (int j = borderWidth - 1 - i; j >= 0; j--) {
			sel.at<uchar>(i, j) = 0;
			sel.at<uchar>(sel.rows - 1 - i, j) = 0;
			sel.at<uchar>(i, sel.cols - 1 - j) = 0;
			sel.at<uchar>(sel.rows - 1 - i, sel.cols - 1 - j) = 0;

		}
	}

	return sel;
}


