#include <stonefish_boids/OctoCoral.hpp>

OctoCoralNode::OctoCoralNode(/* args */)
{
    std::cout << "Building coral node" << "\n";
}

OctoCoralNode::~OctoCoralNode(/* args */)
{
    std::cout << "Destructing coral node" << "\n";
}

void OctoCoralNode::setForce(Eigen::Vector3f force)
{
}

// ###########################################################33333333333333

OctoCoralTree::OctoCoralTree(double resolution) : octomap::OccupancyOcTreeBase<OctoCoralNode>(resolution)
{
    std::cout << "Creating OctoCoralTree with resolution: " << resolution << "\n";
}

OctoCoralTree::~OctoCoralTree()
{
    std::cout << "OctocoralTree destructor" << "\n";
}