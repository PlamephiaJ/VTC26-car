#include "motion_planning/rrt_star_planner.hpp"

#include "motion_planning/occupancy_grid.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace rrt_star
{
namespace
{

using Clock = std::chrono::steady_clock;

class ScopedProfile
{
public:
    explicit ScopedProfile(std::chrono::nanoseconds& destination)
        : destination_(destination), start_(Clock::now()) {}

    ~ScopedProfile()
    {
        destination_ += Clock::now() - start_;
    }

private:
    std::chrono::nanoseconds& destination_;
    Clock::time_point start_;
};

void validate_config(const PlannerConfig& config)
{
    if (config.minimum_iterations <= 0 ||
        config.maximum_iterations < config.minimum_iterations ||
        config.sample_standard_deviation <= 0.0 || config.step_size <= 0.0 ||
        config.near_radius <= 0.0 || config.goal_tolerance <= 0.0 ||
        config.goal_sample_rate < 0.0 || config.goal_sample_rate > 1.0 ||
        config.static_margin < 0.0 || config.dynamic_margin < 0.0)
    {
        throw std::invalid_argument("Invalid RRT* planner configuration");
    }
}

/** SoA storage for fields touched in nearest/near/rewiring hot loops. */
class PlannerTree
{
public:
    explicit PlannerTree(const std::size_t capacity)
    {
        xs_.reserve(capacity);
        ys_.reserve(capacity);
        costs_.reserve(capacity);
        parents_.reserve(capacity);
        children_.reserve(capacity);
        roots_.reserve(capacity);
    }

    std::size_t add(const Node& node)
    {
        const std::size_t index = size();
        xs_.emplace_back(node.x);
        ys_.emplace_back(node.y);
        costs_.emplace_back(node.cost);
        parents_.emplace_back(node.parent);
        children_.emplace_back(node.children);
        roots_.emplace_back(node.is_root);
        return index;
    }

    std::size_t size() const { return xs_.size(); }
    double x(const std::size_t index) const { return xs_[index]; }
    double y(const std::size_t index) const { return ys_[index]; }
    double cost(const std::size_t index) const { return costs_[index]; }
    std::size_t parent(const std::size_t index) const { return parents_[index]; }
    bool is_root(const std::size_t index) const { return roots_[index]; }

    double squared_distance_to(
        const std::size_t index, const double query_x,
        const double query_y) const
    {
        const double dx = xs_[index] - query_x;
        const double dy = ys_[index] - query_y;
        return dx * dx + dy * dy;
    }

    double edge_length(const std::size_t first, const std::size_t second) const
    {
        return std::hypot(xs_[first] - xs_[second], ys_[first] - ys_[second]);
    }

    double edge_length_to(
        const std::size_t first, const double other_x,
        const double other_y) const
    {
        return std::hypot(xs_[first] - other_x, ys_[first] - other_y);
    }

    std::size_t nearest(const Point2D& query) const
    {
        if (size() == 0)
        {
            throw std::invalid_argument("nearest requires a non-empty tree");
        }
        std::size_t nearest_index = 0;
        double minimum = std::numeric_limits<double>::max();
        for (std::size_t i = 0; i < size(); ++i)
        {
            const double candidate = squared_distance_to(i, query.x, query.y);
            if (candidate < minimum)
            {
                minimum = candidate;
                nearest_index = i;
            }
        }
        return nearest_index;
    }

    std::vector<std::size_t> near(
        const double query_x, const double query_y,
        const double radius) const
    {
        const double radius_squared = radius * radius;
        std::vector<std::size_t> result;
        for (std::size_t i = 0; i < size(); ++i)
        {
            if (squared_distance_to(i, query_x, query_y) < radius_squared)
            {
                result.emplace_back(i);
            }
        }
        return result;
    }

    void attach(
        const std::size_t node, const std::size_t parent_index,
        const double node_cost)
    {
        parents_[node] = parent_index;
        costs_[node] = node_cost;
        children_[parent_index].emplace_back(node);
    }

    void reparent(
        const std::size_t node, const std::size_t new_parent,
        const double new_cost)
    {
        auto& old_children = children_[parents_[node]];
        old_children.erase(
            std::remove(old_children.begin(), old_children.end(), node),
            old_children.end());
        parents_[node] = new_parent;
        costs_[node] = new_cost;
        children_[new_parent].emplace_back(node);
        update_descendant_costs(node);
    }

    std::vector<Node> trace_path(std::size_t terminal) const
    {
        std::vector<Node> path;
        while (!roots_[terminal])
        {
            path.emplace_back(copy_node(terminal));
            terminal = parents_[terminal];
        }
        path.emplace_back(copy_node(terminal));
        std::reverse(path.begin(), path.end());
        return path;
    }

    Tree materialize() const
    {
        Tree result;
        result.reserve(size());
        for (std::size_t i = 0; i < size(); ++i)
        {
            result.emplace_back(copy_node(i));
        }
        return result;
    }

private:
    Node copy_node(const std::size_t index) const
    {
        Node result;
        result.x = xs_[index];
        result.y = ys_[index];
        result.cost = costs_[index];
        result.parent = parents_[index];
        result.children = children_[index];
        result.is_root = roots_[index];
        return result;
    }

    void update_descendant_costs(const std::size_t index)
    {
        for (const std::size_t child : children_[index])
        {
            costs_[child] = costs_[index] + edge_length(index, child);
            update_descendant_costs(child);
        }
    }

    std::vector<double> xs_;
    std::vector<double> ys_;
    std::vector<double> costs_;
    std::vector<std::size_t> parents_;
    std::vector<std::vector<std::size_t>> children_;
    std::vector<bool> roots_;
};

bool edge_is_blocked(
    const nav_msgs::msg::OccupancyGrid& dynamic_map,
    const nav_msgs::msg::OccupancyGrid& static_map,
    const double start_x, const double start_y, const bool start_is_root,
    const double end_x, const double end_y,
    const PlannerConfig& config)
{
    geometry_msgs::msg::Point start;
    start.x = start_x;
    start.y = start_y;
    geometry_msgs::msg::Point end;
    end.x = end_x;
    end.y = end_y;

    occupancy_grid::SegmentCheckOptions options;
    if (start_is_root)
    {
        options.escape_reference_map = &static_map;
        options.start_escape_distance =
            std::max(config.static_margin, config.dynamic_margin) +
            dynamic_map.info.resolution;
    }
    return occupancy_grid::segment_is_blocked(dynamic_map, start, end, options);
}

}  // namespace

Planner::Planner(
    const PlannerConfig& config,
    const std::optional<std::uint32_t> random_seed)
    : config_(config),
      random_generator_(random_seed.value_or(std::random_device{}()))
{
    validate_config(config_);
}

std::optional<Point2D> Planner::sample_free_point(
    const Point2D& start, const Point2D& goal,
    const nav_msgs::msg::OccupancyGrid& dynamic_map)
{
    if (unit_distribution_(random_generator_) < config_.goal_sample_rate &&
        !occupancy_grid::is_xy_coord_occupied(dynamic_map, goal.x, goal.y))
    {
        return goal;
    }

    const double center_x = 0.6 * goal.x + 0.4 * start.x;
    const double center_y = 0.6 * goal.y + 0.4 * start.y;
    std::normal_distribution<double> x_distribution(
        center_x, config_.sample_standard_deviation);
    std::normal_distribution<double> y_distribution(
        center_y, config_.sample_standard_deviation);

    constexpr int maximum_attempts = 1000;
    for (int attempt = 0; attempt < maximum_attempts; ++attempt)
    {
        const Point2D sample{
            x_distribution(random_generator_), y_distribution(random_generator_)};
        if (!occupancy_grid::is_xy_coord_occupied(dynamic_map, sample.x, sample.y))
        {
            return sample;
        }
    }
    return std::nullopt;
}

PlanResult Planner::plan(
    const Point2D& start, const Point2D& goal,
    const nav_msgs::msg::OccupancyGrid& dynamic_map,
    const nav_msgs::msg::OccupancyGrid& static_map)
{
    const auto total_start = Clock::now();
    PlanResult result;
    PlannerTree tree(
        static_cast<std::size_t>(config_.maximum_iterations) + 1U);
    Node root;
    root.x = start.x;
    root.y = start.y;
    root.parent = 0;
    root.cost = 0.0;
    root.is_root = true;
    tree.add(root);

    const auto finish = [&]()
    {
        result.tree = tree.materialize();
        result.profile.total = Clock::now() - total_start;
    };

    std::vector<std::size_t> goal_candidates;
    for (int iteration = 0; iteration < config_.maximum_iterations; ++iteration)
    {
        std::optional<Point2D> sample;
        {
            ScopedProfile timer(result.profile.sampling);
            sample = sample_free_point(start, goal, dynamic_map);
        }
        if (!sample)
        {
            result.failure = PlanFailure::sampling_failed;
            break;
        }

        std::size_t nearest = 0;
        double new_x = 0.0;
        double new_y = 0.0;
        {
            ScopedProfile timer(result.profile.nearest);
            nearest = tree.nearest(*sample);
            const double dx = sample->x - tree.x(nearest);
            const double dy = sample->y - tree.y(nearest);
            const double distance = std::hypot(dx, dy);
            if (distance <= std::numeric_limits<double>::epsilon())
            {
                new_x = tree.x(nearest);
                new_y = tree.y(nearest);
            }
            else
            {
                const double travelled = std::min(config_.step_size, distance);
                new_x = tree.x(nearest) + travelled * dx / distance;
                new_y = tree.y(nearest) + travelled * dy / distance;
            }
        }

        bool initial_blocked = false;
        {
            ScopedProfile timer(result.profile.initial_collision);
            if (tree.edge_length_to(nearest, new_x, new_y) <=
                std::numeric_limits<double>::epsilon())
            {
                initial_blocked = true;
            }
            else
            {
                initial_blocked = edge_is_blocked(
                    dynamic_map, static_map,
                    tree.x(nearest), tree.y(nearest), tree.is_root(nearest),
                    new_x, new_y, config_);
            }
        }
        if (initial_blocked)
        {
            continue;
        }

        std::vector<std::size_t> near_nodes;
        {
            ScopedProfile timer(result.profile.near);
            near_nodes = tree.near(new_x, new_y, config_.near_radius);
        }

        std::size_t best_parent = nearest;
        double best_cost =
            tree.cost(nearest) + tree.edge_length_to(nearest, new_x, new_y);
        {
            ScopedProfile timer(result.profile.parent_collision);
            for (const std::size_t candidate : near_nodes)
            {
                if (edge_is_blocked(
                        dynamic_map, static_map,
                        tree.x(candidate), tree.y(candidate),
                        tree.is_root(candidate), new_x, new_y, config_))
                {
                    continue;
                }
                const double candidate_cost = tree.cost(candidate) +
                    tree.edge_length_to(candidate, new_x, new_y);
                if (candidate_cost < best_cost)
                {
                    best_parent = candidate;
                    best_cost = candidate_cost;
                }
            }
        }

        Node new_node;
        new_node.x = new_x;
        new_node.y = new_y;
        const std::size_t new_index = tree.add(new_node);
        tree.attach(new_index, best_parent, best_cost);

        {
            ScopedProfile timer(result.profile.rewiring);
            for (const std::size_t near_index : near_nodes)
            {
                const double rewired_cost = tree.cost(new_index) +
                    tree.edge_length(new_index, near_index);
                if (rewired_cost >= tree.cost(near_index) ||
                    edge_is_blocked(
                        dynamic_map, static_map,
                        tree.x(near_index), tree.y(near_index),
                        tree.is_root(near_index), tree.x(new_index),
                        tree.y(new_index), config_))
                {
                    continue;
                }
                tree.reparent(near_index, new_index, rewired_cost);
            }
        }

        if (tree.squared_distance_to(new_index, goal.x, goal.y) <
            config_.goal_tolerance * config_.goal_tolerance)
        {
            goal_candidates.emplace_back(new_index);
        }

        if (iteration + 1 >= config_.minimum_iterations &&
            !goal_candidates.empty())
        {
            const std::size_t best = *std::min_element(
                goal_candidates.begin(), goal_candidates.end(),
                [&tree, &goal](const std::size_t first, const std::size_t second)
                {
                    return tree.cost(first) + std::sqrt(
                        tree.squared_distance_to(first, goal.x, goal.y)) <
                        tree.cost(second) + std::sqrt(
                        tree.squared_distance_to(second, goal.x, goal.y));
                });
            result.path = tree.trace_path(best);
            result.goal_candidate_count = goal_candidates.size();
            result.success = true;
            result.failure = PlanFailure::none;
            finish();
            return result;
        }
    }

    result.goal_candidate_count = goal_candidates.size();
    finish();
    return result;
}

}  // namespace rrt_star
