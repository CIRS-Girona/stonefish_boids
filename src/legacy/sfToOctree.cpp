#include <stonefish_boids/Octoscn.hpp>

#include <chrono>
#include <thread>

Octoscn::Octoscn()
{
    std::cout << "default octo constructor" << "\n";
}

Octoscn::~Octoscn()
{
    std::cout << "octo scenario destructor" << "\n";
}

void Octoscn::genOctreeScn(sf::SimulationManager *sim)
{
    auto enti = sim->getEntity("Brain");
    btVector3 bbox_center;
    if (enti != NULL)
    {
        std::cout << "got entity of type " << int(enti->getType()) << "\n";

        sf::Vector3 min;
        sf::Vector3 max;
        enti->getAABB(min, max);

        std::cout << "minX " << min.getX() << " maxX " << max.getX() << "\n";
        std::cout << "minY " << min.getY() << " maxY " << max.getY() << "\n";
        std::cout << "minZ " << min.getZ() << " maxZ " << max.getZ() << "\n";

        bbox_center = (max - min) / 2;

        std::cout << "X " << bbox_center.getX() << "\n";
        std::cout << "Y " << bbox_center.getY() << "\n";
        std::cout << "Z " << bbox_center.getZ() << "\n";
    }

    OcForceTree tree(0.1);
    std::cout << tree.getTreeType() << "\n";

    OcForceTreeNode *node = tree.updateNode(0.0, 0.0, 0.0, true);
    if (node)
    {
        node->setForce(Eigen::Vector3f(1.0, 1.0, 1.0));
        // std::cout << node->getForce() << "\n";
    }

    node = tree.updateNode(0.1, 0.1, 0.1, true);

    auto dnode = tree.search(octomap::point3d(0.1, 0.1, 0.1));
    if (dnode != NULL)
        std::cout << "esta occupied? " << tree.isNodeOccupied(dnode) << "\n";
    else
        std::cout << "it is null" << "\n";

    octomap::OcTreeKey key;
    int depth = 10;
    tree.coordToKeyChecked(0, 3.2, 0, depth, key);
    tree.updateNode(key, true);
    // auto onode = tree.search(key, 10);
    // tree.pruneNode(onode);
    // tree.expandNode(onode);

    auto anode = tree.updateNode(0, 1.6, 0, true);

    // std::cout << tree.keyToCoord(key) << "\n";
    // std::cout << tree.keyToCoord(key, depth) << "\n";

    dnode = tree.search(octomap::point3d(3.0, 3.25, 3.25));
    if (dnode != NULL)
        std::cout << "esta occupied? " << tree.isNodeOccupied(dnode) << "\n";
    else
        std::cout << "it is null" << "\n";

    for (OcForceTree::iterator n = tree.begin(), end = tree.end(); n != end; ++n)
    {
        if (tree.isNodeOccupied(*n))
        {
            std::cout << n.getCoordinate() << "\n";
        }
    }

    // tree.expandNode();

    std::cout << "octoprints" << "\n";
    std::cout << tree.calcNumNodes() << "\n";
    std::cout << tree.getNumLeafNodes() << "\n";
    std::cout << tree.getNodeSize(11) << "\n";

    std::cout << "octoprints" << "\n";

    // Save to file
    // auto suc = tree.writeBinary("custom_octree.bt");
    auto suc = tree.writeBinary("/home/srealper/ros_ws/doc_ws/mymap.bt");

    std::this_thread::sleep_for(std::chrono::seconds(3));

    std::cout << "succ?" << suc << "\n";
}