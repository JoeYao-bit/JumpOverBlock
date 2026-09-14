/***********************************************************
 *
 * @file: rrt_connect.h
 * @breif: Contains the RRT Connect planner class
 * @author: Yang Haodong
 * @update: 2023-1-18
 * @version: 1.0
 *
 * Copyright (c) 2023， Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef RRT_CONNECT_H
#define RRT_CONNECT_H

#include "rrt.h"

namespace global_planner
{
/**
 * @brief Class for objects that plan using the RRT Connect algorithm
 */
class RRTConnect : public RRT
{
public:
  /**
   * @brief  Constructor
   * @param   nx          pixel number in costmap x direction
   * @param   ny          pixel number in costmap y direction
   * @param   resolution  costmap resolution
   * @param   sample_num  andom sample points
   * @param   max_dist    max distance between sample points
   */
  RRTConnect(int nx, int ny, double resolution, freeNav::IS_LINE_COLLISION_FREE_FUNC<int, 2> line_collision_check, int sample_num, double max_dist);

  /**
   * @brief RRT implementation
   * @param costs     costmap
   * @param start     start node
   * @param goal      goal node
   * @param expand    containing the node been search during the process
   * @return tuple contatining a bool as to whether a path was found, and the path
   */
  bool plan(const unsigned char* global_costmap, const Node& start, const Node& goal, std::vector<Node>& path,
            std::vector<Node>& expand);

protected:
  /**
   * @brief convert closed list to path
   * @param boundary  connected node that the boudary of forward and backward
   * @return ector containing path nodes
   */
  std::vector<Node> _convertClosedListToPath(const Node& boundary);

protected:
  // Sampled list forward
  std::unordered_set<Node, NodeIdAsHash, compare_coordinates> sample_list_f_;
  // Sampled list backward
  std::unordered_set<Node, NodeIdAsHash, compare_coordinates> sample_list_b_;
};

    freeNav::Path<2> RRTConnectRimJump(const unsigned char* global_costmap, freeNav::DimensionLength* dim, double resolution,
                                       freeNav::IS_LINE_COLLISION_FREE_FUNC<int, 2> line_collision_check,
                                       const freeNav::Pointi<2>& start, const freeNav::Pointi<2>& target,
                                       int sample_num, double max_dist);

}  // namespace global_planner

#endif