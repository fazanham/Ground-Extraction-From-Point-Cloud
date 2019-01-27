#include "BlockProcessor.h"

CBlockProcessor::CBlockProcessor() :
fpInputFile(NULL),
fname_inputData(NULL)
{
	B_Maximal_Data_Extent = new double[4];

	B_Maximal_Data_Extent[0] = std::numeric_limits<double>::min();
	B_Maximal_Data_Extent[1] = std::numeric_limits<double>::max();
	B_Maximal_Data_Extent[2] = std::numeric_limits<double>::min();
	B_Maximal_Data_Extent[3] = std::numeric_limits<double>::max();
}

CBlockProcessor::CBlockProcessor(const char* input_file_name, double size_grid) :
fpInputFile(NULL),
fname_inputData(NULL),
grid_size(size_grid)
{

	int i = strlen(input_file_name);

	if (i > 0 && input_file_name != NULL) {
		fname_inputData = new char[i + 1];

		strcpy(fname_inputData, input_file_name);
		fname_inputData[i] = '\0';
	}

	B_Maximal_Data_Extent = new double[4];

	B_Maximal_Data_Extent[0] = std::numeric_limits<double>::min();
	B_Maximal_Data_Extent[1] = std::numeric_limits<double>::max();
	B_Maximal_Data_Extent[2] = std::numeric_limits<double>::min();
	B_Maximal_Data_Extent[3] = std::numeric_limits<double>::max();
}

void CBlockProcessor::setFilenames(const char * input_file_name)
{
	int i = strlen(input_file_name);

	if (i > 0 && input_file_name != NULL) {
		fname_inputData = new char[i + 1];

		strcpy(fname_inputData, input_file_name);
		fname_inputData[i] = '\0';
	}
}

bool CBlockProcessor::LasReader()
{
	CBlockProcessor::LasInput = new CLasInput(fname_inputData);
	CBlockProcessor::LasInput->ReadHeader();

	// no. of points
	CBlockProcessor::n = CBlockProcessor::LasInput->file_header.Num_p;

	//Open the rawdata file
	fpInputFile = fopen(fname_inputData, "rb");

	// offset to the data points
	fseek(fpInputFile, CBlockProcessor::LasInput->file_header.OffsetToData, SEEK_SET);

	return true;
}

std::tuple<double **, int> CBlockProcessor::BlockProcess()
{
	// Raw LiDAR information
	long* x_l; // Easting, Northing and Upping scale factor
	long x_o;
	x_l = &x_o;

	unsigned short* x_us; // Intensity scale factor
	unsigned short i_us;
	x_us = &i_us;

	unsigned long num; // Number of returns
	unsigned long *k;
	k = &num;

	double *x_d; // Offset to the next point
	double t_d;
	x_d = &t_d; 

	double ** mat_f = (double **)malloc(B_size * sizeof(double *)); // Array for storing the block points
	for (int j = 0; j < B_size; j++) {
		mat_f[j] = (double *)malloc(sizeof(double) * 3);
	}

	int mat_count = 0;
	int mat_f_count = -1;

	CBlockProcessor::Original_Cloud.clear(); // Clear the original cloud

	for (; CBlockProcessor::BP_current_count < CBlockProcessor::BP_counter*CBlockProcessor::B_size; ++CBlockProcessor::BP_current_count) {
		if (BP_current_count < n) {
			fread(x_l, 4, 1, fpInputFile);
			double pos_x = x_o*LasInput->file_header.x_s + LasInput->file_header.x_offset;

			fread(x_l, 4, 1, fpInputFile);
			double pos_y = x_o*LasInput->file_header.y_s + LasInput->file_header.y_offset;

			fread(x_l, 4, 1, fpInputFile);
			double pos_h = x_o*LasInput->file_header.z_s + LasInput->file_header.z_offset;

			fread(x_us, 2, 1, fpInputFile);
			double raw_intensity = i_us;

			fread(k, 1, 1, fpInputFile);
			std::bitset<8> b(num);
			int NumReturn = b[0] * 1 + b[1] * 2 + b[2] * 4;

			fseek(fpInputFile, 5, 1);
			fread(x_d, 8, 1, fpInputFile);

			if (CBlockProcessor::BP_current_count < (CBlockProcessor::BP_counter*CBlockProcessor::B_size) && pos_h < MAX_HEIGHT_THRESHOLD) {

				CBlockProcessor::Original_Cloud.push_back(cv::Point3f(pos_x, pos_y, pos_h));

				mat_count++;

				if (mat_count > 1) {
					if (NumReturn <= 1) { // We only store the first returns
						mat_f_count++;

						if (mat_f_count == B_size) { //increase array size 
							int size2 = 2 * B_size;
							mat_f = (double **)realloc(mat_f, size2 * sizeof(double *));
							for (int k = mat_f_count; k < size2; k++)
								mat_f[k] = (double *)malloc(sizeof(double) * 5);
						}

						mat_f[mat_f_count][0] = pos_x;
						mat_f[mat_f_count][1] = pos_y;
						mat_f[mat_f_count][2] = pos_h;

						if (B_Maximal_Data_Extent[1] > mat_f[mat_f_count][0])
							B_Maximal_Data_Extent[1] = mat_f[mat_f_count][0];
						if (B_Maximal_Data_Extent[0] < mat_f[mat_f_count][0])
							B_Maximal_Data_Extent[0] = mat_f[mat_f_count][0];
						if (B_Maximal_Data_Extent[3] > mat_f[mat_f_count][1])
							B_Maximal_Data_Extent[3] = mat_f[mat_f_count][1];
						if (B_Maximal_Data_Extent[2] < mat_f[mat_f_count][1])
							B_Maximal_Data_Extent[2] = mat_f[mat_f_count][1];
					}
				}
			}
		}
		else {
			CBlockProcessor::IsPointCloudNotExhausted = false;

			break;
		}
	}

	printf("Points Processed: %d / %d\n", CBlockProcessor::B_size*CBlockProcessor::BP_counter, (CBlockProcessor::n));

	CBlockProcessor::BP_counter++;

	return std::make_tuple(mat_f, mat_f_count);
}