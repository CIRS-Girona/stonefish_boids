#include <stonefish_boids/OcForceTree.hpp>

// OcForceTreeNode::OcForceTreeNode() : force(Eigen::Vector3f::Zero())
OcForceTreeNode::OcForceTreeNode() : force({1, 2, 3})
{
    // std::cout << "Building coral node" << "\n";
}

OcForceTreeNode::~OcForceTreeNode()
{
    // std::cout << "Destructing coral node" << "\n";
}

void OcForceTreeNode::setForce(Eigen::Vector3f fvector)
{
    force = fvector;
}

std::ostream &OcForceTreeNode::writeData(std::ostream &s) const
{

    s.write((const char *)&force, sizeof(force));
    return s;
}

// ###########################################################33333333333333

OcForceTree::OcForceTree(double resolution) : octomap::OccupancyOcTreeBase<OcForceTreeNode>(resolution)
{
    std::cout << "Creating OcForceTree with resolution: " << resolution << "\n";
}

OcForceTree::~OcForceTree()
{
    std::cout << "OcForceTree destructor" << "\n";
}