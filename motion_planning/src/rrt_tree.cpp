#include "motion_planning/rrt_tree.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace rrt_star
{

double squared_distance(const Point2D& first, const Point2D& second)
{
    const double dx = first.x - second.x;
    const double dy = first.y - second.y;
    return dx * dx + dy * dy;
}

double edge_length(const Node& first, const Node& second)
{
    return std::hypot(first.x - second.x, first.y - second.y);
}

std::size_t nearest_index(const Tree& tree, const Point2D& query)
{
    if (tree.empty())
    {
        throw std::invalid_argument("nearest_index requires a non-empty tree");
    }

    std::size_t nearest = 0;
    double minimum_distance = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < tree.size(); ++i)
    {
        const double distance = squared_distance({tree.at(i).x, tree.at(i).y}, query);
        if (distance < minimum_distance)
        {
            minimum_distance = distance;
            nearest = i;
        }
    }
    return nearest;
}

Node steer_towards(const Node& source, const Point2D& sample, const double step_size)
{
    if (step_size <= 0.0)
    {
        throw std::invalid_argument("step_size must be positive");
    }

    Node result;
    const double distance = std::hypot(sample.x - source.x, sample.y - source.y);
    if (distance <= std::numeric_limits<double>::epsilon())
    {
        result.x = source.x;
        result.y = source.y;
        return result;
    }

    const double travelled = std::min(step_size, distance);
    result.x = source.x + travelled * (sample.x - source.x) / distance;
    result.y = source.y + travelled * (sample.y - source.y) / distance;
    return result;
}

std::vector<std::size_t> near_indices(
    const Tree& tree, const Node& query, const double radius)
{
    if (radius <= 0.0)
    {
        throw std::invalid_argument("radius must be positive");
    }

    std::vector<std::size_t> result;
    const double radius_squared = radius * radius;
    const Point2D query_point{query.x, query.y};
    for (std::size_t i = 0; i < tree.size(); ++i)
    {
        const Node& candidate = tree.at(i);
        if (squared_distance({candidate.x, candidate.y}, query_point) < radius_squared)
        {
            result.emplace_back(i);
        }
    }
    return result;
}

bool is_within_goal(const Node& node, const Point2D& goal, const double tolerance)
{
    return squared_distance({node.x, node.y}, goal) < tolerance * tolerance;
}

void update_descendant_costs(Tree& tree, const std::size_t node_index)
{
    for (const std::size_t child_index : tree.at(node_index).children)
    {
        Node& child = tree.at(child_index);
        child.cost = tree.at(node_index).cost + edge_length(tree.at(node_index), child);
        update_descendant_costs(tree, child_index);
    }
}

void reparent_node(
    Tree& tree, const std::size_t node_index, const std::size_t new_parent_index,
    const double new_cost)
{
    Node& node = tree.at(node_index);
    const std::size_t old_parent_index = node.parent;
    auto& old_children = tree.at(old_parent_index).children;
    old_children.erase(
        std::remove(old_children.begin(), old_children.end(), node_index),
        old_children.end());

    node.parent = new_parent_index;
    node.cost = new_cost;
    tree.at(new_parent_index).children.emplace_back(node_index);
    update_descendant_costs(tree, node_index);
}

std::vector<Node> trace_path(const Tree& tree, const std::size_t terminal_index)
{
    std::vector<Node> path;
    std::size_t current_index = terminal_index;
    while (!tree.at(current_index).is_root)
    {
        path.emplace_back(tree.at(current_index));
        current_index = tree.at(current_index).parent;
    }
    path.emplace_back(tree.at(current_index));
    std::reverse(path.begin(), path.end());
    return path;
}

std::size_t best_goal_candidate(
    const Tree& tree, const std::vector<std::size_t>& candidates,
    const Point2D& goal)
{
    if (candidates.empty())
    {
        throw std::invalid_argument("best_goal_candidate requires candidates");
    }

    return *std::min_element(
        candidates.begin(), candidates.end(),
        [&tree, &goal](const std::size_t first, const std::size_t second)
        {
            const Node& first_node = tree.at(first);
            const Node& second_node = tree.at(second);
            const double first_cost = first_node.cost +
                std::sqrt(squared_distance({first_node.x, first_node.y}, goal));
            const double second_cost = second_node.cost +
                std::sqrt(squared_distance({second_node.x, second_node.y}, goal));
            return first_cost < second_cost;
        });
}

}  // namespace rrt_star
