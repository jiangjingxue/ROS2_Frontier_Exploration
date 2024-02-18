# Ros2_Frontier_Detector
![target-with-frontiers-2](https://github.com/jiangjingxue/ROS2_Frontier_Detection/assets/119235879/309635bc-b3f9-4cfb-a593-0da2f09e1cd0)
## Usage
Include `frontier_detector.h` header in your file:
```
#include "frontier_detector.h"
```
Dynamically create a frontier detector object:
```
frontier_detector = std::make_shared<WavefrontFrontierDetector>();
```
Create a vector to hold the detected frontier segements:
```
std::vector<FrontierSegment> frontier_segments;
```
Search for the frontiers:
```
frontier_detector->searchFrontierSegments(frontier_segments,current_robot_pose,costmap);
```
You can access the centroid of the detected frontier segments i by:
```
geometry_msgs::msg::Point centroid = frontier_segments[i].centroid;
```



