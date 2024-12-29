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

#include <stonefish_boids/Boids.hpp>
#include <stonefish_boids/RRT.hpp>
#include <thread>

struct Flock
{
    // Boids boids;
    // std::shared_ptr<Boids> boids;
    std::string flockName;
    Boids boids;
    std::vector<sf::ManualTrajectory *> boidTrajs;
    sf::ManualTrajectory *leaderTraj;
    std::vector<Eigen::Vector3f> leaderPath;
    int leaderPathIndex;
};

class MyManager : public sf::ROSSimulationManager
{
private:
    ros::ServiceClient client;
    ros::ServiceServer service;

    int numBoids = 20;
    std::vector<std::thread> threads;

    std::vector<std::shared_ptr<Flock>> flocks;

    sf::AnimatedEntity *anim;
    // sf::ManualTrajectory *leader_traj;
    // std::vector<sf::ManualTrajectory *> boid_paths;
    // std::vector<Eigen::Vector3f> leaderPath;

    ros::Timer timer;
    void timerCb(const ros::TimerEvent &evt);
    // float inc = 0;

public:
    MyManager(sf::Scalar rate, std::string scenarioPath);
    ~MyManager();

    void UpdateGroup(std::shared_ptr<Flock> flock);
    Eigen::Vector3f randomVector3f(const std::vector<float> &xlim,
                                   const std::vector<float> &ylim,
                                   const std::vector<float> &zlim);
    void MakeFishes();
    void BuildScenario();
};

MyManager::MyManager(sf::Scalar rate, std::string scenarioPath) : ROSSimulationManager(rate, scenarioPath)
{
    std::cout << "inherited rosmanager" << "\n";
    std::cout << rate << "\n";
    std::cout << scenarioPath << "\n";

    // leader_traj = new sf::ManualTrajectory();
    timer = nh.createTimer(ros::Duration(1.0 / 30.0), &MyManager::timerCb, this);
}

MyManager::~MyManager()
{
    std::cout << "killing fishes" << "\n";
    for (auto &th : threads)
    {
        if (th.joinable())
        {
            th.join();
        }
    }
}

void MyManager::UpdateGroup(std::shared_ptr<Flock> flock)
{
    // std::cout << flock->flockName << "\n";

    if (flock->leaderPathIndex > flock->leaderPath.size() - 1)
    {
        std::reverse(flock->leaderPath.begin(), flock->leaderPath.end());
        flock->leaderPathIndex = 0;
    }

    flock->boids.setLeaderPosition(flock->leaderPath.at(flock->leaderPathIndex));
    flock->leaderPathIndex++;

    // flock->boids.setLeaderPosition(Vector3f(0, 0, 2));
    flock->boids.updateBoids(1.0 / 30.0);
    auto leader_pos = flock->boids.getLeaderPosition();
    sf::Vector3 point(leader_pos.x(), leader_pos.y(), leader_pos.z());
    sf::Quaternion ori(0, 0, 0, 1);
    flock->leaderTraj->setTransform(sf::Transform(ori, point));

    for (int i = 0; i < flock->boids.getBoids().size(); i++)
    {
        auto boid = flock->boids.getBoids().at(i);
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

        flock->boidTrajs.at(i)->setTransform(sf::Transform(ori, point));
    }
}

void MyManager::timerCb(const ros::TimerEvent &evt)
{

    // crear vector de threads aca?
    threads.clear();
    for (auto flock : flocks)
    {
        threads.push_back(std::thread(&MyManager::UpdateGroup, this, flock));
    }
    for (auto &th : threads)
    {
        th.join();
    }
}

Eigen::Vector3f MyManager::randomVector3f(const std::vector<float> &xlim,
                                          const std::vector<float> &ylim,
                                          const std::vector<float> &zlim)
{

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> disx(xlim[0], xlim[1]);
    std::uniform_real_distribution<double> disy(ylim[0], ylim[1]);
    std::uniform_real_distribution<double> disz(zlim[0], zlim[1]);

    return Eigen::Vector3f(disx(gen), disy(gen), disz(gen));
}

void MyManager::MakeFishes()
{
    Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols,
                                 ", ", ", ", "", "", " << ", ";");

    XmlRpc::XmlRpcValue boids_cfg;
    bool gotten = nh.getParam("/boids_settings", boids_cfg);
    std::cout << gotten << "\n";
    // int b = 0;
    for (int b = 0; b < boids_cfg.size(); b++)
    {
        std::cout << "found: " << boids_cfg[b]["name"] << "\n";
        // Flock flock;
        auto flock = std::make_shared<Flock>();
        flock->flockName = std::string(boids_cfg[b]["name"]);
        numBoids = static_cast<int>(boids_cfg[b]["numboids"]);
        // flock->boids = std::make_unique<Boids>(numBoids, Vector3f(0, 0, 0));
        Eigen::Vector3f initPoint = (randomVector3f({-5, 5}, {-5, 5}, {2, 5}));
        flock->boids = Boids(numBoids, initPoint);
        std::cout << static_cast<double>(boids_cfg[b]["alignmentWeight"]) << "\n";
        flock->boids.setAlignmentWeight(static_cast<double>(boids_cfg[b]["alignmentWeight"]));
        flock->boids.setCohesionWeight(static_cast<double>(boids_cfg[b]["cohesionWeight"]));
        flock->boids.setSeparationWeight(static_cast<double>(boids_cfg[b]["separationWeight"]));
        flock->boids.setLeaderAttractionWeight(static_cast<double>(boids_cfg[b]["leaderAttractionWeight"]));
        flock->boids.setMaxSpeed(static_cast<double>(boids_cfg[b]["maxSpeed"]));
        flock->boids.setPerceptionRadius(static_cast<double>(boids_cfg[b]["perceptionRadius"]));
        flock->boids.setSeparationRadius(static_cast<double>(boids_cfg[b]["separationRadius"]));
        flock->boids.printSettings();

        RRT rrt;
        Eigen::Vector3f endPoint = (randomVector3f({-10, 10}, {-10, 10}, {2, 7}));
        std::cout << "initPoint" << initPoint.format(CommaInitFmt) << "\n";
        std::cout << "endPoint" << endPoint.format(CommaInitFmt) << "\n";
        flock->leaderPath = rrt.doPlan(initPoint, endPoint);
        flock->leaderTraj = new sf::ManualTrajectory();
        anim = new sf::AnimatedEntity("LeaderFrame" + std::to_string(b), flock->leaderTraj);
        AddAnimatedEntity(anim);

        flocks.push_back(flock);
    }

    // CreateLook("fishtex", sf::Color::Gray(1.f), 0.1f, 0.f, 0.f,
    //            sf::GetDataPath() + "creatures/yellowtang/yellowtang.png");
    // create control ptr for every boid
    int b = 0;
    for (auto flock : flocks)
    {
        CreateLook(flock->flockName + "tex", sf::Color::Gray(1.f), 0.1f, 0.f, 0.f,
                   sf::GetDataPath() + std::string(boids_cfg[b]["texturePath"]));

        for (int i = 0; i < flock->boids.getBoids().size(); i++)
        {
            sf::ManualTrajectory *traj = new sf::ManualTrajectory();
            flock->boidTrajs.push_back(traj);

            anim = new sf::AnimatedEntity("AnimMesh" + std::to_string(i), flock->boidTrajs.at(i),
                                          sf::GetDataPath() + std::string(boids_cfg[b]["modelPath"]),
                                          1.0, sf::I4(),
                                          sf::GetDataPath() + std::string(boids_cfg[b]["modelPath"]),
                                          1.0, sf::I4(), "Neutral", flock->flockName + "tex");
            AddAnimatedEntity(anim);
        }
        b++;
    }

    std::cout << flocks.size() << "\n";
}

void MyManager::BuildScenario()
{
    // Run parser
    sf::ROSScenarioParser parser(this);
    bool success = parser.Parse(scnFilePath);

    // Save log
    std::string logFilePath = ros::file_log::getLogDirectory() + "/stonefish_ros_parser.log";
    bool success2 = parser.SaveLog(logFilePath);

    if (!success)
    {
        ROS_ERROR("Parsing of scenario file '%s' failed!", scnFilePath.c_str());
        if (success2)
            ROS_ERROR("For more information check the parser log file '%s'.", logFilePath.c_str());
    }

    if (!success2)
        ROS_ERROR("Parser log file '%s' could not be saved!", logFilePath.c_str());

    // Create object
    CreateLook("red", sf::Color::RGB(1.f, 0.f, 0.f), 0.1f, 0.f);
    sf::Scalar angle = M_PI / 180.0 * 14.1;
    sf::BodyPhysicsSettings phy;
    phy.mode = sf::BodyPhysicsMode::DISABLED;
    phy.collisions = true;
    sf::Box *box = new sf::Box("Box", phy, sf::Vector3(0.1, 0.1, 0.1), sf::I4(), "Steel", "red");
    AddSolidEntity(box, sf::Transform(sf::Quaternion(0, angle, 0), sf::Vector3(2.5, 0, -1.72)));

    MakeFishes();

    spinner.start();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "parsed_simulator", ros::init_options::NoSigintHandler);

    // Check number of command line arguments
    if (argc < 7)
    {
        ROS_FATAL("Not enough command line arguments provided!");
        return 1;
    }

    // Parse arguments
    std::string dataDirPath = std::string(argv[1]) + "/";
    std::string scenarioPath(argv[2]);

    std::cout << "resource path: " << dataDirPath << "\n";
    std::cout << "scenario path: " << scenarioPath << "\n";

    sf::Scalar rate = atof(argv[3]);

    sf::RenderSettings s;
    s.windowW = atoi(argv[4]);
    s.windowH = atoi(argv[5]);

    std::string quality(argv[6]);
    if (quality == "low")
    {
        s.shadows = sf::RenderQuality::LOW;
        s.ao = sf::RenderQuality::DISABLED;
        s.atmosphere = sf::RenderQuality::LOW;
        s.ocean = sf::RenderQuality::LOW;
        s.aa = sf::RenderQuality::LOW;
        s.ssr = sf::RenderQuality::DISABLED;
    }
    else if (quality == "high")
    {
        s.shadows = sf::RenderQuality::HIGH;
        s.ao = sf::RenderQuality::HIGH;
        s.atmosphere = sf::RenderQuality::HIGH;
        s.ocean = sf::RenderQuality::HIGH;
        s.aa = sf::RenderQuality::HIGH;
        s.ssr = sf::RenderQuality::HIGH;
    }
    else // "medium"
    {
        s.shadows = sf::RenderQuality::MEDIUM;
        s.ao = sf::RenderQuality::MEDIUM;
        s.atmosphere = sf::RenderQuality::MEDIUM;
        s.ocean = sf::RenderQuality::MEDIUM;
        s.aa = sf::RenderQuality::MEDIUM;
        s.ssr = sf::RenderQuality::MEDIUM;
    }

    sf::HelperSettings h;
    h.showFluidDynamics = false;
    h.showCoordSys = false;
    h.showBulletDebugInfo = false;
    h.showSensors = false;
    h.showActuators = false;
    h.showForces = false;

    // sf::ROSSimulationManager manager(rate, scenarioPath);

    MyManager *manager = new MyManager(rate, scenarioPath);
    sf::GraphicalSimulationApp app("Stonefish Simulator", dataDirPath, s, h, manager);

    app.Run();

    return 0;
}
