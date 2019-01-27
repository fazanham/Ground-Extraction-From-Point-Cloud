# include "LasInput.h"

CLasInput::CLasInput() :
	InputFile(NULL),
	fname_envParameters(NULL)
{}

CLasInput::CLasInput(const char *input_file_name) :
	InputFile(NULL),
	fname_envParameters(NULL)
{
	int i = strlen(input_file_name);

	if (i > 0 && input_file_name != NULL)
	{
		fname_envParameters = new char[i + 1];
		strcpy(fname_envParameters, input_file_name);
		fname_envParameters[i] = '\0';
	}
}

CLasInput::~CLasInput()
{
	if (fname_envParameters != NULL)
	{
		delete[]fname_envParameters;
		fname_envParameters = NULL;
	}
}

CLasInput::file_header_type::file_header_type() :
	OffsetToData(0),
	DataFormat(NULL),
	Num_p(0),
	x_s(0.0),
	y_s(0.0),
	z_s(0.0),
	x_offset(0.0),
	y_offset(0.0),
	z_offset(0.0),
	max_x(0.0),
	min_x(0.0),
	max_y(0.0),
	min_y(0.0),
	max_z(0.0),
	min_z(0.0)
{}

CLasInput::file_header_type::file_header_type(const CLasInput::file_header_type &s) :
	OffsetToData(s.OffsetToData),
	DataFormat(s.DataFormat),
	Num_p(s.Num_p),
	x_s(s.x_s),
	y_s(s.y_s),
	z_s(s.z_s),
	x_offset(s.x_offset),
	y_offset(s.y_offset),
	z_offset(s.z_offset),
	max_x(s.max_x),
	min_x(s.min_x),
	max_y(s.max_y),
	min_y(s.min_y),
	max_z(s.max_z),
	min_z(s.min_z)
{}

void CLasInput::file_header_type::clear()
{
	OffsetToData = 0;
	DataFormat = NULL;
	Num_p = 0;
	x_s = 0.0;
	y_s = 0.0;
	z_s = 0.0;
	x_offset = 0.0;
	y_offset = 0.0;
	z_offset = 0.0;
	max_x = 0.0;
	min_x = 0.0;
	max_y = 0.0;
	min_y = 0.0;
	max_z = 0.0;
	min_z = 0.0;
}

bool CLasInput::SetFileName(const char *input_file_name)
{
	int i = strlen(input_file_name);

	if (i > 0 && fname_envParameters == NULL)
	{
		fname_envParameters = new char[i + 1];
		strcpy(fname_envParameters, input_file_name);
		fname_envParameters[i] = '\0';
	}

	return true;
}

bool CLasInput::GetEnvFileName(char *strCopied)
{
	int str_len = strlen(fname_envParameters) + 1;

	if (str_len > 1)
	{
		strCopied = new char[str_len];
		strcpy(strCopied, fname_envParameters);
		strCopied[str_len - 1] = '\0';
	}
	return true;
}
// Reads the header
bool CLasInput::ReadHeader()
{
	InputFile = fopen(fname_envParameters, "rb");

	// Read the offset to point data:
	fseek(InputFile, 96, 0);
	unsigned long *offset = &file_header.OffsetToData;
	fread(offset, 4, 1, InputFile);

	// Read the data format ID:
	fseek(InputFile, 4, 1);
	unsigned char *format = &file_header.DataFormat;
	fread(format, 1, 1, InputFile);

	// Read the number of point records:
	fseek(InputFile, 2, 1);
	unsigned long* num_points = &file_header.Num_p;
	fread(num_points, 4, 1, InputFile);

	// Read the scale factors:
	fseek(InputFile, 20, 1);
	double* x_d;

	x_d = &file_header.x_s;
	fread(x_d, 8, 1, InputFile);
	// Fread automatically moves the file pointer.
	x_d = &file_header.y_s;
	fread(x_d, 8, 1, InputFile);
	x_d = &file_header.z_s;
	fread(x_d, 8, 1, InputFile);
	x_d = &file_header.x_offset;
	fread(x_d, 8, 1, InputFile);
	x_d = &file_header.y_offset;
	fread(x_d, 8, 1, InputFile);
	x_d = &file_header.z_offset;
	fread(x_d, 8, 1, InputFile);
	x_d = &file_header.max_x;
	fread(x_d, 8, 1, InputFile);
	x_d = &file_header.min_x;
	fread(x_d, 8, 1, InputFile);
	x_d = &file_header.max_y;
	fread(x_d, 8, 1, InputFile);
	x_d = &file_header.min_y;
	fread(x_d, 8, 1, InputFile);
	x_d = &file_header.max_z;
	fread(x_d, 8, 1, InputFile);
	x_d = &file_header.min_z;
	fread(x_d, 8, 1, InputFile);

	fflush(InputFile);
	fclose(InputFile);

	return true;
}

//Retrieves the header information
bool CLasInput::GetHeader(file_header_type *header)
{
	header->OffsetToData = file_header.OffsetToData;
	header->DataFormat = file_header.DataFormat;
	header->Num_p = file_header.Num_p;
	header->x_s = file_header.x_s;
	header->y_s = file_header.y_s;
	header->z_s = file_header.z_s;
	header->x_offset = file_header.x_offset;
	header->y_offset = file_header.y_offset;
	header->z_offset = file_header.z_offset;
	header->max_x = file_header.max_x;
	header->min_x = file_header.min_x;
	header->max_y = file_header.max_y;
	header->min_y = file_header.min_y;
	header->max_z = file_header.max_z;
	header->min_z = file_header.min_z;

	return true;

}