#pragma once

/*
* LasInput.h
* A class for reading las files (supported format LAS 1.2)
*/

#ifndef __LASINPUT_H__
#define __LASINPUT_H__

#include "LH.h"
#define MAX_STRLEN 200

class CLasInput
{
public:
	CLasInput();
	CLasInput(const char * input_file_name);
	~CLasInput();

	struct file_header_type
	{
		unsigned long OffsetToData;
		unsigned char DataFormat;
		unsigned long Num_p;
		double x_s;
		double y_s;
		double z_s;
		double x_offset;
		double y_offset;
		double z_offset;
		double max_x;
		double min_x;
		double max_y;
		double min_y;
		double max_z;
		double min_z;

		file_header_type();
		file_header_type(const file_header_type &s);
		~file_header_type() {};
		void clear();
	};
	file_header_type file_header;

	bool ReadHeader();
	bool SetFileName(const char* input_file_name);
	bool GetEnvFileName(char* strCopied);
	bool GetHeader(file_header_type* header);


private:

	char* fname_envParameters;
	FILE* InputFile;

};

#endif
