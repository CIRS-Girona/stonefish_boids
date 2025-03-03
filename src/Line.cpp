#include <stonefish_boids/Line.hpp>
#include <iostream>

Line::Line()
{

    std::cout << "\033[;34m\n";
    std::cout << "Loading Line planner\n";
    // create a state space

    std::cout << "Loaded Line planner\n";
    std::cout << "\033[0m" << "\n";
}

std::vector<Eigen::Vector3f> Line::doPlan(Eigen::Vector3f start, Eigen::Vector3f goal)
{
    std::cout << "\033[;34m\n";
    std::cout << "Generating path\n";

    Eigen::Vector3f dir = (goal - start);
    float len = dir.norm();
    dir.stableNormalize();
    float step = 0.05;

    for (float s = 0; s < len; s += step)
    {
        auto point = start + (dir * s);
        path_.push_back(point);
    }

    // goal
    path_.at(path_.size() - 1) = goal;

    std::cout << "Path generated\n";
    std::cout << "\033[0m" << "\n";

    return path_;
}
