//
// Created by jingxue on 13/08/23.
//
#include "frontier_detector.h"

#include <queue>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace orca
{
    // constructor
    WavefrontFrontierDetector::WavefrontFrontierDetector() {

    }
    void WavefrontFrontierDetector::searchFrontierSegments(
            std::vector<FrontierSegment>& frontier_segments,
            const geometry_msgs::msg::Point current_robot_position_world,
            const nav2_costmap_2d::Costmap2D& map){

        // std::vector<FrontierSegment> frontier_segments;     // a vector that contains all the found frontier segments

        // 1.Convert current robot pose from world coordinates to map coordinates
        // 2.while checking for legal bounds, return false if the world coordinates is out of bounds
        unsigned int robot_pos_x,robot_pos_y;
        if(!map.worldToMap(current_robot_position_world.x,current_robot_position_world.y,robot_pos_x,robot_pos_y)){
            RCLCPP_WARN(rclcpp::get_logger("frontier_detector"), "Robot is out of cost map bounds, cannot search for frontier segments");
            return;
        }

        // TODO: Make sure map is locked during the search

        // 1. Get map size
        auto size_xy = map.getSizeInCellsX() * map.getSizeInCellsY();

        // 1. In order to avoid rescanning the same map point and
        // 2. detecting the same frontier reachable from two frontier
        // 3. points, WFD marks map points with four indications:
        std::vector<unsigned char> cell_states_m(size_xy,0);

        // 1. create a queue for outermost BFS
        std::queue<unsigned int> queue_m;

        // 1. Given two map coordinates , compute the associated index
        // 2. Essentially computing the robot's current position in index form
        unsigned int robot_pos_idx = map.getIndex(robot_pos_x,robot_pos_y);

        // 1. Enqueue robot's current position to outermost queue_m and mark it as visited (label it as MAP OPEN LIST)
        queue_m.push(robot_pos_idx);
        cell_states_m[robot_pos_idx] = MapOpenList;   // TODO: check for rescanned points

        while(!queue_m.empty()){
            unsigned int p = queue_m.front();
            queue_m.pop();

            // if current cell is already scanned, skip this iteration
            if(cell_states_m[p] == MapCloseList)
                continue;

            // if current cell is a frontier cell , extract frontier segment from this cell
            // create a new frontier segment object
            // save the extracted frontier segment
            // mark all points in the frontier segment as MAP CLOSE LIST
            if(isFrontierCell(p,map)){
                // TODO: extract frontier segment from the frontier cell
                FrontierSegment new_frontier_segment = FrontierSegment();
                extractNewFrontierSegmentFrom(new_frontier_segment,p,cell_states_m,map);

                // SAVE START AND END POINT
                geometry_msgs::msg::Point start_point, end_point;
                start_point = new_frontier_segment.frontier_points.front();
                end_point = new_frontier_segment.frontier_points.back();
                new_frontier_segment.endpoints[0] = start_point;
                new_frontier_segment.endpoints[1] = end_point;
                if(new_frontier_segment.size >= 8){
                    frontier_segments.push_back(new_frontier_segment);
                }


                // NOTE: mark all points of New Frontier Segment as MAP CLOSE LIST
                // is done inside the function extractNewFrontierSegmentFrom()
            }

            // enqueue any neighbors of the current cell that is not visited and has at least one free neighbors
            for(const auto neighbor_idx: getNeighbors_8(p,map)){
                if(cell_states_m[neighbor_idx] != MapCloseList && cell_states_m[neighbor_idx] != MapOpenList  && hasFreeNeighbor(neighbor_idx,map)){
                    queue_m.push(neighbor_idx);
                    cell_states_m[neighbor_idx] = MapOpenList;
                }
            }

            // mark current cell as MAP CLOSE LIST
            cell_states_m[p] = MapCloseList;

        }
        // return;
    }

    /**
     * @brief for now, keep only the largest frontier segments;
     * @param frontier_segments
     * @return
     */
    void WavefrontFrontierDetector::filterFrontierSegments(std::vector<FrontierSegment>& frontier_segments){
        // find the two largest frontier segment
        std::sort(frontier_segments.begin(),frontier_segments.end(),
                  [](const FrontierSegment& s1, const FrontierSegment& s2){
            return s1.size > s2.size;
        });

        frontier_segments.erase(frontier_segments.begin()+1,frontier_segments.end());
    }

    /**
     * @function extractFrontierSegmentFromFrontierCell
     * @brief extract new frontier segment from a found frontier cell.
     * @brief A frontier segment consist of all frontier points and properties related to the segment such as size
     * @param frontier_cell_idx
     * @param map
     * @param new_frontier_segment
     * @returns void
     */
    void WavefrontFrontierDetector::extractNewFrontierSegmentFrom(FrontierSegment& new_frontier_segment, unsigned int frontier_cell_idx, std::vector<unsigned char>& cell_state_map, const nav2_costmap_2d::Costmap2D& map){

        // 1. create a vector to mark cells (is it necessary to create another list?)
        auto size_xy = map.getSizeInCellsX() * map.getSizeInCellsY();
        std::vector<unsigned char> cell_states_frontier(size_xy,0);

        // 1. create a queue for inner BFS
        std::queue<unsigned int> queue_f;

        // 1. Enqueue frontier cell to queue_f and mark it as visited (label it as FRONTIER OPEN LIST)
        queue_f.push(frontier_cell_idx);
        cell_states_frontier[frontier_cell_idx] = FrontierOpenList;

        while(!queue_f.empty()){
            unsigned int q = queue_f.front();
            queue_f.pop();

            // check if current cell is already scanned
            if(cell_states_frontier[q] == FrontierCloseList || cell_state_map[q] == MapCloseList)
                continue;

            // if current cell is a frontier cell
            // add current cell to the frontier segment and modify frontier segment's size
            if(isFrontierCell(q,map)){
                geometry_msgs::msg::Point point;
                convertIndextoPoint(point,q,map);
                new_frontier_segment.frontier_points.push_back(point);
                new_frontier_segment.size++;
                // update the centroid
                new_frontier_segment.centroid.x += point.x;
                new_frontier_segment.centroid.y += point.y;

                // get all neighbors of the current frontier cell
                // if the neighbor has not been visited before, enqueue it and mark it as FRONTIER OPEN LIST
                for(const auto neighbor_idx: getNeighbors_8(q,map)){
                    if(cell_states_frontier[neighbor_idx] != FrontierOpenList
                    && cell_states_frontier[neighbor_idx] != FrontierCloseList
                    && cell_state_map[neighbor_idx] != MapCloseList){
                        queue_f.push(neighbor_idx);
                        cell_states_frontier[neighbor_idx] = FrontierOpenList;
                    }
                }

                // mark all points of New Frontier Segment as MAP CLOSE LIST
                cell_state_map[q] = MapCloseList;
            }

            // mark current cell q as a FRONTIER CLOSE LIST
            cell_states_frontier[q] = FrontierCloseList;
        }

        // average out the centroid
        new_frontier_segment.centroid.x = new_frontier_segment.centroid.x / (double)new_frontier_segment.size;
        new_frontier_segment.centroid.y = new_frontier_segment.centroid.y / (double)new_frontier_segment.size;

        // return new_frontier_segment;
    }

    /**
     * @function isFrontierCell
     * @brief determine if the input cell is a frontier cell. Options: 4 or 8 connectivity
     * @brief frontier cell: an unknown cell that has at least one neighbor that is known AND free
     * @brief or Frontiers are defined as free-space cells in an occupancy grid that have at least one unknown cell as a neighbour
     * @param idx: index of the input cell
     * @param map: cost map
     * @returns a boolean value indicating if the input cell is a frontier cell
     */
    bool WavefrontFrontierDetector::isFrontierCell(unsigned int idx, const nav2_costmap_2d::Costmap2D& map){
        // 1) early return if the input cell is not an unknown cell
        // 2) get the neighbors of the input cell, return true if any one of the neighbors is free
        // 3) no need to check empty vector because std::any_of takes care of for it

        if(map.getCost(idx) != nav2_costmap_2d::FREE_SPACE)
            return false;

        std::vector<unsigned int> neighbors_indices = getNeighbors_8(idx,map);   // modify this line if you need 8-connectivity


//        size_t num_unknown_neighbors = 0;
//        return std::any_of(neighbors_indices.cbegin(),neighbors_indices.cend(),[&map,&num_unknown_neighbors](unsigned int x){
//            if(map.getCost(x) == nav2_costmap_2d::NO_INFORMATION){
//                num_unknown_neighbors++;
//            }
//            if(num_unknown_neighbors < 3)
//                return false;
//            else
//                return true;
//        });
        return std::any_of(neighbors_indices.cbegin(),neighbors_indices.cend(),[&map](unsigned int x){
            return map.getCost(x) == nav2_costmap_2d::NO_INFORMATION;
        });

    }


    /**
    * @function getNeighbors4
    * @brief get the 4-connected neighbors of a cell
    * @param center_idx: index of the current/center cell
    * @param map: reference to the cost map
    * @returns vector of neighbors' indices
    */
    std::vector<unsigned int> WavefrontFrontierDetector::getNeighbors_4(unsigned int center_idx, const nav2_costmap_2d::Costmap2D &map) {
        // 1) reserve memory for the vector
        // 2) check map's size, return if center_idx is invalid/out of bound
        // 3) check for top edge, left edge, right edge, bottom edge

        std::vector<unsigned int> out;
        out.reserve(4);

        size_t width = map.getSizeInCellsX();
        size_t height = map.getSizeInCellsY();

        if (center_idx > width * height - 1) {
            RCLCPP_WARN(rclcpp::get_logger("frontier_detector"), "Trying to access neighbors of out of bound cell");
            return out;
        }

        unsigned int col, row;
        map.indexToCells(center_idx, col, row);

        // add top neighbor if the cell is not in the first row
        if (row > 0)
            out.push_back(center_idx - width);

        // add left neighbor if the cell is not in the leftmost column
        if (col > 0)
            out.push_back(center_idx - 1);

        // add right neighbor if the cell is not in the rightmost column
        if (col < width - 1)
            out.push_back(center_idx + 1);

        // add bottom neighbor if the cell is not in the bottom row
        if (row < height - 1)
            out.push_back(center_idx + width);

        return out;
    }

    /**
    * @function getNeighbors8
    * @brief get the 8-connected neighbors of a cell
    * @param center_idx: index of the current/center cell
    * @param map: reference to the cost map
    * @returns vector of neighbors' indices
    */
    std::vector<unsigned int> WavefrontFrontierDetector::getNeighbors_8(unsigned int center_idx, const nav2_costmap_2d::Costmap2D &map) {
        std::vector<unsigned int> out;
        out.reserve(8);

        size_t width = map.getSizeInCellsX();
        size_t height = map.getSizeInCellsY();

        if (center_idx > width * height - 1) {
            RCLCPP_WARN(rclcpp::get_logger("frontier_detector"), "Trying to access neighbors of out of bound cell");
            return out;
        }

        unsigned int col, row;
        map.indexToCells(center_idx, col, row);

        // 1.add top left corner neighbor if the cell is not located in the first row and the first column
        if (row > 0 && col > 0)
            out.push_back(center_idx - width - 1);

        // 2.add top neighbor if the cell is not in the first row
        if (row > 0)
            out.push_back(center_idx - width);

        // 3.add top right corner neighbor if the cell is not located in the top row and the rightmost column
        if (row > 0 && col < width - 1)
            out.push_back(center_idx - width + 1);

        // 4.add left neighbor if the cell is not in the leftmost column
        if (col > 0)
            out.push_back(center_idx - 1);

        // 5.add right neighbor if the cell is not in the rightmost column
        if (col < width - 1)
            out.push_back(center_idx + 1);

        // 6.add bottom left neighbor if the cell is not located in the bottom row and leftmost column
        if (row < height - 1 && col > 0)
            out.push_back(center_idx + width - 1);

        // 7.add bottom neighbor if the cell is not in the bottom row
        if (row < height - 1)
            out.push_back(center_idx + width);

        // 8.add bottom right neighbor if the cell is not located in the bottom row and the rightmost column
        if (row < height - 1 && col < width - 1)
            out.push_back(center_idx + width + 1);

        return out;
    }

    /**
     * @function hasFreeNeighbor
     * @brief check if the input cell has at least one free neighbor
     * @param idx: index
     * @param map: cost map
     * @return true if input cell has at least one free neighbor
     */
    bool WavefrontFrontierDetector::hasFreeNeighbor(unsigned int idx, const nav2_costmap_2d::Costmap2D& map){
        std::vector<unsigned int> neighbors_indices = getNeighbors_8(idx,map);   // modify this line if you need 8-connectivity

        return std::any_of(neighbors_indices.cbegin(),neighbors_indices.cend(),[&](unsigned int x){
            return map.getCost(x) == nav2_costmap_2d::FREE_SPACE;
        });
    }

    /**
     * @function covert an frontier cell index to a geometry point
     * @param point
     * @param frontier_cell_idx
     */
    void WavefrontFrontierDetector::convertIndextoPoint(geometry_msgs::msg::Point& point,unsigned int frontier_cell_idx,const nav2_costmap_2d::Costmap2D &map){
        unsigned int mx;
        unsigned int my;
        map.indexToCells(frontier_cell_idx,mx,my);

        double wx;
        double wy;
        map.mapToWorld(mx,my,wx,wy);

        point.x = wx;
        point.y = wy;
    }

}