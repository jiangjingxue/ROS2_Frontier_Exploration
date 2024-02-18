# Ros2_Frontier_Detector
## How to
Include `frontier_detector.h` header in your file
```
#include "frontier_detector.h"
```
Dynamically create a frontier detector object
```
frontier_detector = std::make_shared<WavefrontFrontierDetector>();
```
Create a vector to hold the detected frontier segements
```
std::vector<FrontierSegment> frontier_segments;
```
Search for the frontiers
```
frontier_detector->searchFrontierSegments(frontier_segments,current_robot_pose,costmap);
```
You can access the centroid of the detected frontier segments i by:
```
geometry_msgs::msg::Point centroid = frontier_segments[i].centroid;
```



