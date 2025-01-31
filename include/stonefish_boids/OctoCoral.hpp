#ifndef OCTOCORAL_H
#define OCTOCORAL_H

#include <octomap/octomap.h>
#include <Eigen/Eigen>

class OctoCoralNode : public octomap::OcTreeNode
{
private:
    /* data */
public:
    OctoCoralNode(/* args */);
    ~OctoCoralNode();

    void setForce(Eigen::Vector3f force);

    Eigen::Vector3f getForce();
};

class OctoCoralTree : public octomap::OccupancyOcTreeBase<OctoCoralNode>
{
private:
    /* data */
public:
    OctoCoralTree(double resolution);
    ~OctoCoralTree();

    std::string getTreeType() const { return "OctoCoralTree"; }

    OctoCoralTree *create() const { return new OctoCoralTree(resolution); }
};

#endif
