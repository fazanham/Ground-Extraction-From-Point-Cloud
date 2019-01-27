#pragma once

/*
* BlockProcessor.h
* Class for block processing the point cloud for faster speed.
* Created on: September, 2017
* Author: Faizaan Naveed
*/

#ifndef BLOCKPROCESSOR_H
#define BLOCKPROCESSOR_H

#include "LasInput.h"
#include "LH.h"

#define MAX_HEIGHT_THRESHOLD 200 // The maximum threshold height to remove outliers from the point cloud

class CBlockProcessor {

public:

	CBlockProcessor();
	CBlockProcessor(const char* input_file_name, double size_grid);

	// ATTRIBUTES

	FILE* fpInputFile; // File containing the point cloud data 
	CLasInput *LasInput; // Instance for reading the lasReader class

	double* B_Maximal_Data_Extent; // The maximal spatial extent of the data
	bool IsPointCloudNotExhausted = true;
	
	std::vector<cv::Point3f> Original_Cloud; // The original point cloud
	int n; // Number of the total detected points in the cloud
	const int B_size = 100000; // The maximum number of points in each block.

	// METHODS

	/* Set the filenames for the Las and flight file
	@param input_file_name: Las file
	@param flight_file_name: Flight file
	*/
	void setFilenames(const char* input_file_name);

	/*Open the las file and extract the header information*/
	bool LasReader(); 

	/*Incrementally read the las file for block processing the cloud*/
	std::tuple<double**, int> BlockProcess();

private:

	// ATTRIBUTES

	char* fname_inputData; //Name of the file containing raw data
	size_t grid_size;

	// Parameters for the block
	int BP_current_count = 0; // Count for the number of LiDAR points that have been already processed
	int BP_count = 0;
	int BP_counter = 1; // number of blocks

	double size_grid; // size of the grid
};

#endif