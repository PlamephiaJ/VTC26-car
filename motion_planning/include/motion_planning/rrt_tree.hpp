#ifndef MOTION_PLANNING__RRT_TREE_HPP_
#define MOTION_PLANNING__RRT_TREE_HPP_

#include <cstddef>
#include <vector>

namespace rrt_star
{

/** A planar point used by the ROS-independent RRT* core. */
struct Point2D
{
    double x = 0.0;
    double y = 0.0;
};

/**
 * One node in an RRT* tree.
 *
 * `cost` is the accumulated distance from the root. `parent` and `children`
 * are indices into the owning Tree. The root uses parent index zero.
 */
struct Node
{
    double x = 0.0;
    double y = 0.0;
    double cost = 0.0;
    std::size_t parent = 0;
    std::vector<std::size_t> children;
    bool is_root = false;
};

using Tree = std::vector<Node>;

/**
 * Compute squared planar distance.
 *
 * Input: two planar coordinates.
 * Return: squared Euclidean distance; no square root is evaluated.
 */
double squared_distance(const Point2D& first, const Point2D& second);

/**
 * Compute planar distance between two nodes.
 *
 * Input: two valid nodes.
 * Return: Euclidean distance in map units.
 */
double edge_length(const Node& first, const Node& second);

/**
 * Locate the tree node closest to a query point.
 *
 * Input: a non-empty tree and a planar query point.
 * Return: index of the closest node. Throws std::invalid_argument for an
 * empty tree.
 */
std::size_t nearest_index(const Tree& tree, const Point2D& query);

/**
 * Move from a node toward a sample by at most `step_size`.
 *
 * Input: source node, sampled point, and positive maximum step length.
 * Return: a detached node at the steered coordinate; parent/cost are left at
 * their defaults for the planner to assign.
 */
Node steer_towards(const Node& source, const Point2D& sample, double step_size);

/**
 * Find nodes within a strict radius of a query node.
 *
 * Input: tree, query node, and positive radius.
 * Return: tree indices whose Euclidean distance is smaller than the radius.
 */
std::vector<std::size_t> near_indices(
    const Tree& tree, const Node& query, double radius);

/**
 * Test whether a node lies inside the goal tolerance.
 *
 * Input: node, goal coordinate, and positive tolerance radius.
 * Return: true when the node-to-goal distance is strictly below tolerance.
 */
bool is_within_goal(const Node& node, const Point2D& goal, double tolerance);

/**
 * Change a node's parent while keeping parent/child links consistent.
 *
 * Input: mutable tree, existing node index, new parent index, and the node's
 * already-computed new accumulated cost.
 * Operation: removes the node from its old parent's children, attaches it to
 * the new parent, and recursively refreshes all descendant costs.
 */
void reparent_node(
    Tree& tree, std::size_t node_index, std::size_t new_parent_index,
    double new_cost);

/**
 * Recompute accumulated costs below one node.
 *
 * Input: mutable tree and a valid root-of-subtree index.
 * Operation: recursively sets every descendant cost from its current parent.
 */
void update_descendant_costs(Tree& tree, std::size_t node_index);

/**
 * Reconstruct a root-to-node path using parent links.
 *
 * Input: tree and a valid terminal node index.
 * Return: copied nodes ordered from root to terminal node.
 */
std::vector<Node> trace_path(const Tree& tree, std::size_t terminal_index);

/**
 * Choose the lowest total-cost goal candidate.
 *
 * Input: tree, candidate node indices, and exact goal coordinate.
 * Return: candidate minimizing root cost plus straight-line remainder to the
 * goal. Throws std::invalid_argument when candidates are empty.
 */
std::size_t best_goal_candidate(
    const Tree& tree, const std::vector<std::size_t>& candidates,
    const Point2D& goal);

}  // namespace rrt_star

#endif  // MOTION_PLANNING__RRT_TREE_HPP_
