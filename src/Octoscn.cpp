#include <stonefish_boids/Octoscn.hpp>

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
    if (enti != NULL)
    {
        std::cout << "got entity of type " << int(enti->getType()) << "\n";

        sf::Vector3 min;
        sf::Vector3 max;
        enti->getAABB(min, max);

        std::cout << min.getX() << "\n";
        std::cout << max.getX() << "\n";

        octomap::OcTree octree(0.1);
        //  octree.setNodeValue();
        // octree.updateNode()

        OctoCoralTree tree(0.1);
        std::cout << tree.getTreeType() << "\n";
    }
}