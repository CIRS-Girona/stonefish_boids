#include <stonefish_boids/OcForceTree.hpp>
#include <memory>

class CollisionEnv
{
private:
public:
    CollisionEnv(/* args */);
    ~CollisionEnv();

    std::shared_ptr<OcForceTree> collisionTree;
};
