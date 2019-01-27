# Ground-Extraction-From-Point-Cloud
This is a software for finely removing non-ground points from point clouds. The software consists of two major steps: 

1) Coarse Removal: The program coarsely filters out non-ground points in the point cloud using a morphological operation with disk structuring element. Although this step removes almost all of the non-ground points, it also ends up removing some of the ground points.

2) Refinement: To compensate for the removed ground points we triangulate the coarse approximation from step 1. The program then iteratively examines the points in the original cloud to check if they are ground points. If the point is a ground point and does not exist in the coarse approximation, it is added to it.

# Installation
The project requires a few open-source libraries. I have provided the libraries along with the project so you do not have to go through the trouble of finding them:

OpenCV (v3.1)
PCL (v1.8)
Fade2.5D (v1.73)

3rdParty libraries used by PCL
VTK (v7.0)
Boost (v1.61)
Flann

## Setup
You can find a detailed tutorial on setting up OpenCV within your Visual Studio environment here:
https://www.deciphertechnic.com/install-opencv-with-visual-studio/

For installing PCL, you can check out their page. I have linked the pre-built binaries in the uploaded project so you can use those directly, but if you want to browse their source code you can find more details here:
http://www.pointclouds.org/downloads/

Fade2.5D is a great library for performing delaunay triangulation operations (2D, 2.5D and 3D). You can check out their page here for more information:
http://www.geom.at/fade2d/html/

## Project Properties
Inside Properties of your project,

1. Go to C/C++ > General. Copy the path to include folders of the libraries and paste it inside Additional Include Directories. The path will look similar to C:\opencv\build\include. Then, click Apply.

2. Go to linker > General. Copy the path to folders containing the  lib files and paste it inside Additional Library Directories. The path will look similar to C:\opencv\build\x64\vc14\lib. Then, click Apply.

3. Go to linker > Input > Additional Dependencies. Add the following lib file: 

opencv_world310d.lib
pcl_common_debug.lib
pcl_segmentation_debug.lib
pcl_features_debug.lib
pcl_filters_debug.lib
pcl_io_debug.lib
pcl_io_ply_debug.lib
pcl_kdtree_debug.lib
pcl_visualization_debug.lib
pcl_surface_debug.lib
pcl_search_debug.lib
OpenNI2.lib
vtkalglib-7.0.lib
vtkRenderingCore-7.0-gd.lib
fade25D_x64_v140_Debug.lib
fade2D_x64_v140_Debug.lib
tiff.lib
