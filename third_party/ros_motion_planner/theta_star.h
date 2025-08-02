/***********************************************************
 *
 * @file: theta_star.h
 * @breif: Contains the Theta* planner class
 * @author: Wu Maojia, Yang Haodong
 * @update: 2023-8-26
 * @version: 1.1
 *
 * Copyright (c) 2023， Wu Maojia
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef THETA_STAR_H
#define THETA_STAR_H

#include <vector>
#include <queue>
#include "global_planner.h"

namespace global_planner
{
/**
 * @brief Class for objects that plan using the Theta* algorithm
 */
class ThetaStar : public GlobalPlanner
{
public:
  /**
   * @brief Construct a new ThetaStar object
   * @param nx          pixel number in costmap x direction
   * @param ny          pixel number in costmap y direction
   * @param resolution  costmap resolution
   */
  ThetaStar(int nx, int ny, double resolution, freeNav::IS_LINE_COLLISION_FREE_FUNC<2> line_collision_check);

  /**
   * @brief Theta* implementation
   * @param global_costmap global costmap
   * @param start         start node
   * @param goal          goal node
   * @param path          optimal path consists of Node
   * @param expand        containing the node been search during the process
   * @return  true if path found, else false
   */
  bool plan(const unsigned char* global_costmap, const Node& start, const Node& goal, std::vector<Node>& path,
            std::vector<Node>& expand);

protected:
  /**
   * @brief update the g value of child node
   * @param parent
   * @param child
   */
  void _updateVertex(const Node& parent, Node& child);

  /**
   * @brief Bresenham algorithm to check if there is any obstacle between parent and child
   * @param parent
   * @param child
   * @param costs global costmap
   * @return true if no obstacle, else false
   */
  bool _lineOfSight(const Node& parent, const Node& child, const unsigned char* costs);

private:
  const unsigned char* costs_;  // costmap copy
};

freeNav::Path<2> ThetaStarRimJump(const unsigned char* global_costmap, freeNav::DimensionLength* dim,
                                  freeNav::IS_LINE_COLLISION_FREE_FUNC<2> line_collision_check,
                                  const freeNav::Pointi<2>& start, const freeNav::Pointi<2>& target);

}  // namespace global_planner
#endif
