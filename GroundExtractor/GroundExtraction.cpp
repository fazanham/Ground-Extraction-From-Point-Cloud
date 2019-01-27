#include "TerrainFilter.h"

int main(int argv, char * argc[]) {

	if (argv < 2) {
		fprintf(stderr, "Usage: %s, Input Filename\n", argc[0]);
		exit(-1);
	}

	int str_len = strlen(argc[1]);
	char *filename = new char[str_len + 1];

	memset(filename, 0x00, str_len + 1);
	strcpy(filename, argc[1]);

	FILE *fMS = fopen(filename, "rb");
	if (fMS == NULL) {
		fprintf(stderr, "Error opening file\n");
		//system("PAUSE");
		exit(-1);
	}
	else {

		double grid_size = 0.25;
		int disk_radius_for_morphological_operation = 28;
		double height_difference_for_coarsely_removing_non_ground_points = 0.65;

		CTerrainFilter *CFT = new CTerrainFilter(argc[1], grid_size, disk_radius_for_morphological_operation, height_difference_for_coarsely_removing_non_ground_points);

		int Num_points_for_computing_mean = 7;
		double num_of_std_dev_for_removing_outliers = 0.95;

		double Angle_threshold_for_point_addition = 0.57; // angle in radians
		double Distance_threshold_for_point_addition = 0.25;

		boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> Triangulator = CFT->Triangulation(Num_points_for_computing_mean, num_of_std_dev_for_removing_outliers, 
			Angle_threshold_for_point_addition, Distance_threshold_for_point_addition);

		CFT->writeData2ascii("C:\\Users\\Faizaan\\Documents\\Visual Studio 2015\\Projects\\GroundExtractor\\Output\\GroundPoints2.txt" ,Triangulator);
	}

	return 0;
}