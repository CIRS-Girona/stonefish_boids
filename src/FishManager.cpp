#include <ros/ros.h>
#include <ros/file_log.h>
#include <Stonefish/core/GraphicalSimulationApp.h>
#include <Stonefish/utils/SystemUtil.hpp>
#include <stonefish_ros/ROSSimulationManager.h>
#include <stonefish_ros/ROSScenarioParser.h>

#include <Stonefish/entities/SolidEntity.h>
#include <Stonefish/entities/solids/Sphere.h>
#include <Stonefish/entities/solids/Box.h>
#include <Stonefish/entities/AnimatedEntity.h>
#include <Stonefish/entities/animation/ManualTrajectory.h>

#include <cirs_boids/Flock.hpp>

class FishManager
{
private:
    sf::ROSSimulationManager *sim;
    const int numBoids = 20;
    const double worldSize = 2.0;
    const double dt = 0.1;
    std::unique_ptr<Flock> flock;
    std::vector<Flock> groups;

    sf::AnimatedEntity *anim;
    sf::ManualTrajectory *leader_traj;
    std::vector<sf::ManualTrajectory *> boid_paths;

    ros::Timer timer;
    void timerCb(const ros::TimerEvent &evt);

public:
    FishManager(sf::ROSSimulationManager *sim);
    ~FishManager();

    void SpawnFishes();
    void UpdateGroup();
};

FishManager::FishManager(sf::ROSSimulationManager *sim)
{
    std::cout << "inherited rosmanager" << "\n";

    leader_traj = new sf::ManualTrajectory();
    timer = nh.createTimer(ros::Duration(1.0 / 30.0), &FishManager::timerCb, this);
}

FishManager::~FishManager()
{
}

void FishManager::UpdateGroup()
{
}

void FishManager::timerCb(const ros::TimerEvent &evt)
{

    flock->setLeaderPosition(Vector3f(0, 0, 2));
    flock->updateBoids(1.0 / 30.0);
    auto leader_pos = flock->getLeaderPosition();
    sf::Vector3 point(leader_pos.x(), leader_pos.y(), leader_pos.z());
    sf::Quaternion ori(0, 0, 0, 1);
    leader_traj->setTransform(sf::Transform(ori, point));

    for (int i = 0; i < flock->getBoids().size(); i++)
    {
        auto boid = flock->getBoids().at(i);
        sf::Vector3 point(boid.position.x(), boid.position.y(), boid.position.z());

        // Orientation as quaternion with no roll
        Vector3f forward = boid.velocity.normalized();   // Direction of movement
        Vector3f up = Vector3f::UnitZ();                 // Fixed "up" direction
        Vector3f right = up.cross(forward).normalized(); // Perpendicular right vector
        up = forward.cross(right).normalized();          // Recompute orthogonal up vector
        Eigen::Matrix3f rotationMatrix;
        rotationMatrix.col(0) = forward; // Forward direction
        rotationMatrix.col(1) = right;   // Right direction
        rotationMatrix.col(2) = up;      // Up direction
        Eigen::Quaternionf orientation(rotationMatrix);
        sf::Quaternion ori(orientation.x(), orientation.y(),
                           orientation.z(), orientation.w());

        boid_paths.at(i)->setTransform(sf::Transform(ori, point));
    }
}

void FishManager::SpawnFishes()
{

    flock = std::make_unique<Flock>(numBoids, worldSize, Vector3f(0, 0, 0));

    XmlRpc::XmlRpcValue boids_cfg;
    bool gotten = nh.getParam("/boids_settings", boids_cfg);
    std::cout << gotten << "\n";
    int b = 0;
    for (; b < boids_cfg.size(); b++)
    {
        std::cout << "found: " << boids_cfg[b]["name"] << "\n";
        std::cout << boids_cfg[b]["name"] << "\n";
        std::cout << static_cast<double>(boids_cfg[b]["alignmentWeight"]) << "\n";
        flock->setAlignmentWeight(static_cast<double>(boids_cfg[b]["alignmentWeight"]));
        flock->setCohesionWeight(static_cast<double>(boids_cfg[b]["cohesionWeight"]));
        flock->setSeparationWeight(static_cast<double>(boids_cfg[b]["separationWeight"]));
        flock->setLeaderAttractionWeight(static_cast<double>(boids_cfg[b]["leaderAttractionWeight"]));
        flock->setMaxSpeed(static_cast<double>(boids_cfg[b]["maxSpeed"]));
        flock->setPerceptionRadius(static_cast<double>(boids_cfg[b]["perceptionRadius"]));
        flock->setSeparationRadius(static_cast<double>(boids_cfg[b]["separationRadius"]));

        flock->printSettings();
        break;
    }

    CreateLook("fishtex", sf::Color::Gray(1.f), 0.1f, 0.f, 0.f,
               sf::GetDataPath() + "creatures/yellowtang/yellowtang.png");
    // create control ptr for every boid
    for (int i = 0; i < numBoids; i++)
    {
        sf::ManualTrajectory *traj = new sf::ManualTrajectory();
        boid_paths.push_back(traj);

        anim = new sf::AnimatedEntity("AnimMesh" + std::to_string(i), boid_paths.at(i),
                                      sf::GetDataPath() + "creatures/yellowtang/yellowtang.obj",
                                      1.0, sf::I4(),
                                      sf::GetDataPath() + "creatures/yellowtang/yellowtang.obj",
                                      1.0, sf::I4(), "Neutral", "fishtex");
        AddAnimatedEntity(anim);
    }

    std::cout << boid_paths.size() << "\n";
    anim = new sf::AnimatedEntity("LeaderFrame", leader_traj);
    AddAnimatedEntity(anim);
}
