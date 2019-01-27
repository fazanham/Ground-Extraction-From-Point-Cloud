# Ground-Extraction-From-Point-Cloud
This is a software for finely removing non-ground points from point clouds. The software consists of two major steps: 

1) Coarse Removal: The program coarsely filters out non-ground points in the point cloud using a morphological operation with disk structuring element. Although this step removes almost all of the non-ground points, it also ends up removing some of the ground points.

2) Refinement: To compensate for the removed ground points we triangulate the coarse approximation from step 1. The program then iteratively examines the points in the original cloud to check if they are ground points. If the point is a ground point and does not exist in the coarse approximation, it is added to it.

# Installation
The project requires a few open-source libraries. 

OpenCV (v3.1)<br/>
PCL (v1.8)<br/>
Fade2.5D (v1.73)

3rdParty libraries used by PCL<br/>
VTK (v7.0)<br/>
Boost (v1.61)<br/>
Flann

# Setup
You can find a detailed tutorial on setting up OpenCV within your Visual Studio environment here:
https://www.deciphertechnic.com/install-opencv-with-visual-studio/

For installing PCL, you can check out their page. They provide pre-built binaries, but if you want to play around where their source code you can do that as well.
http://www.pointclouds.org/downloads/

Fade2.5D is a great library for performing delaunay triangulation operations (2D, 2.5D and 3D). You can check out their page here for more information:
http://www.geom.at/fade2d/html/

# Project Properties
Inside Properties of your project,

1. Go to C/C++ > General. Copy the path to include folders of the libraries and paste it inside Additional Include Directories. The path will look similar to C:\opencv\build\include. Then, click Apply.

2. Go to linker > General. Copy the path to folders containing the  lib files and paste it inside Additional Library Directories. The path will look similar to C:\opencv\build\x64\vc14\lib. Then, click Apply.

3. Go to linker > Input > Additional Dependencies. Add the lib files here. Then, click Apply.

# Usage
To run the program, you can use the GroundExtraction.cpp file. The program uses another program called BlockProcessor to process the point cloud block by block. Each block is limited to a maximum of 100,000 points for optimization purposes. Each block is processed to initially coarsely remove non-ground points using a morphological operation and then the approximation is refined via TIN densification. At the end each block is stitched together to form the final cloud with only the ground points.

# Example
Original Point Cloud:
![data](https://user-images.githubusercontent.com/33495209/51795574-bfaa9180-21b3-11e9-952e-d18928fefa71.JPG)

Ground Points:
![ground](https://user-images.githubusercontent.com/33495209/51795579-d2bd6180-21b3-11e9-8462-2739e2009e56.JPG)


## License
Free-to-use (MIT), but at your own risk.

## Credits
Xiaoqian Zhao, Qinghua Guo, Yanjun Su, Baolin Xue,<br/>
Improved progressive TIN densification filtering algorithm for airborne LiDAR data in forested areas,<br/>
ISPRS Journal of Photogrammetry and Remote Sensing,<br/>
Volume 117,<br/>
2016,<br/>
Pages 79-91,<br/>
ISSN 0924-2716,<br/>
https://doi.org/10.1016/j.isprsjprs.2016.03.016.<br/>
