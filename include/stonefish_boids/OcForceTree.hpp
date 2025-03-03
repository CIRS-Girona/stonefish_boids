#ifndef OCFORCETREE_H
#define OCFORCETREE_H

#include <octomap/octomap.h>
#include <Eigen/Eigen>

class OcForceTreeNode : public octomap::OcTreeNode
{
private:
    Eigen::Vector3f force;

public:
    OcForceTreeNode(/* args */);
    ~OcForceTreeNode();

    void setForce(Eigen::Vector3f force);
    Eigen::Vector3f getForce() const { return force; }

    std::ostream &writeData(std::ostream &s) const;
};

// ###########################################################33333333333333

class OcForceTree : public octomap::OccupancyOcTreeBase<OcForceTreeNode>
{
private:
    /* data */
public:
    OcForceTree(double resolution);
    ~OcForceTree();

    std::string getTreeType() const { return "OcForceTree"; }

    OcForceTree *create() const { return new OcForceTree(resolution); }
};

#endif
