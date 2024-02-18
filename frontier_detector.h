//
// Created by jingxue on 13/08/23.
//

#ifndef BUILD_FRONTIER_DETECTOR_H
#define BUILD_FRONTIER_DETECTOR_H

#include <vector>
#include "geometry_msgs/msg/point.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace orca
{
    struct FrontierSegment{
        FrontierSegment() : size(0){

        }
        size_t size;
        double cost;
        geometry_msgs::msg::Point representative_point;
        geometry_msgs::msg::Point centroid;
        std::array<geometry_msgs::msg::Point,2> endpoints;
        std::vector<geometry_msgs::msg::Point> frontier_points;
    };

    struct FilteringParams{
        size_t min_frontier_segment_size;
    };

    enum ListType{
        MapCloseList = 1,
        MapOpenList = 2,
        FrontierCloseList = 4,
        FrontierOpenList = 8
    };

    class WavefrontFrontierDetector{
    public:
        WavefrontFrontierDetector();
        void searchFrontierSegments(std::vector<FrontierSegment>& frontier_segments,geometry_msgs::msg::Point current_robot_pose, const nav2_costmap_2d::Costmap2D& map);
        void filterFrontierSegments(std::vector<FrontierSegment>& frontier_segments);
    private:
        void extractNewFrontierSegmentFrom(FrontierSegment& new_frontier_segment, unsigned int frontier_cell_idx, std::vector<unsigned char>& cell_state_map, const nav2_costmap_2d::Costmap2D& map);
        bool isFrontierCell(unsigned int idx, const nav2_costmap_2d::Costmap2D& map);
        std::vector<unsigned int> getNeighbors_4(unsigned int center_idx, const nav2_costmap_2d::Costmap2D &map);
        std::vector<unsigned int> getNeighbors_8(unsigned int center_idx, const nav2_costmap_2d::Costmap2D &map);
        bool hasFreeNeighbor(unsigned int idx, const nav2_costmap_2d::Costmap2D& map);
        void convertIndextoPoint(geometry_msgs::msg::Point& point,unsigned int frontier_cell_idx,const nav2_costmap_2d::Costmap2D &map);

        // parameters (frontier segments selection criteria)
        FilteringParams filtering_params_{};
    };
}

#endif //BUILD_FRONTIER_DETECTOR_H
