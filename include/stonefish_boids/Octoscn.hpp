// #include <octomap/octomap.h>
#include <Stonefish/core/SimulationManager.h>
#include <memory>

#include <stonefish_boids/OctoCoral.hpp>

class Octoscn
{
private:
    /* data */
    sf::SimulationManager *siman;
    // std::shared_ptr<sf::SimulationManager> siman;

public:
    Octoscn(/* args */);
    ~Octoscn();

    void genOctreeScn(sf::SimulationManager *sim);
};
