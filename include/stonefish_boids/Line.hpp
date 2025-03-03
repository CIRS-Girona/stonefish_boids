#ifndef BOIDSLINE_
#define BOIDSLINE_

#include <vector>
#include <Eigen/Core>

class Line
{
private:
public:
    Line();
    // ~Line();

    std::vector<Eigen::Vector3f> path_;
    std::vector<Eigen::Vector3f> doPlan(Eigen::Vector3f start, Eigen::Vector3f goal);
};

#endif