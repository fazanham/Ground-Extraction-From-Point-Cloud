# Ground-Extraction-From-Point-Cloud
This is a software for finely removing non-ground points from point clouds. The software consists of two major steps: 

1) Coarse Removal: The program coarsely filters out non-ground points in the point cloud using a morphological operation with disk structuring element. Although this step removes almost all of the non-ground points, it also ends up removing some of the ground points.

2) Refinement: To compensate for the removed ground points we triangulate the coarse approximation from step 1. The program then iteratively examines the points in the original cloud to check if they are ground points. If the point is a ground point and does not exist in the coarse approximation, it is added to it.

# Installation
The project requires a few open-source libraries. I have provided the libraries along with the project so you do not have to go through the trouble:
OpenCV (v3.1)
Eigen (v3.3.4)
