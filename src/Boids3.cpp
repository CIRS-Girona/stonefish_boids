#include <stonefish_boids/Boids3.hpp>

#include <iostream>
#include <vector>
#include <random>
#include <Eigen/Core>
#include <chrono>

using Eigen::Vector3f;

Boid3::Boid3(u_int id) : id(id)
{
}

Boid3::~Boid3()
{
}

void Boid3::addExternalForce(const Vector3f extForce, const double weight)
{
    force += (extForce * weight);
    // std::cout << force << "\n";
}

// Alignment: Match the velocity of nearby boids
// Cohesion: Steer towards the average position of nearby boids
// Separation: Avoid crowding nearby boids
Vector3f Boids3::computeRules(const Boid3 &boid)
{
    // aligment vars
    Vector3f avgVelocity = Vector3f::Zero();
    // cohesion vars
    Vector3f avgPosition = Vector3f::Zero();
    // repulsion vars
    Vector3f repulsion = Vector3f::Zero();
    int count = 0;

    int cx = floor(boid.position.x() / cell_size);
    int cy = floor(boid.position.y() / cell_size);
    int cz = floor(boid.position.z() / cell_size);
    // std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    for (int dx = -1; dx <= 1; dx++)
    {
        for (int dy = -1; dy <= 1; dy++)
        {
            for (int dz = -1; dz <= 1; dz++)
            {
                int h = hash(cx + dx, cy + dy, cz + dz);
                // make sure the hash is real
                if (spatial_grid.find(h) != spatial_grid.end())
                {
                    // all boid ids in this hash
                    for (u_int &other_id : spatial_grid[h])
                    {
                        double distance = (boid.position - boids[other_id].position).norm();
                        if (distance < perceptionRadius && &boid != &boids[other_id])
                        {
                            // aligment rule
                            avgVelocity += boids[other_id].velocity;
                            // cohesion rule
                            avgPosition += boids[other_id].position;
                            count++;
                        }
                        // separation rule
                        if (distance < separationRadius && &boid != &boids[other_id])
                        {
                            repulsion -= (boids[other_id].position - boid.position) / distance;
                        }
                    }
                }
            }
        }
    }

    // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    // std::cout << "iteration took = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[us]" << std::endl;
    // std::cout << name << ": boid " << boid.id << " number of neighbors: " << count << "\n";
    if (count > 0)
    {
        // aligment + cohesion + separation
        return ((avgVelocity / count).normalized() * alignmentWeight) +
               (((avgPosition / count) - boid.position).normalized() * cohesionWeight) +
               (repulsion.normalized() * separationWeight);
    }
    else
    {
        return Vector3f::Zero() + repulsion.normalized();
    }
}

Vector3f Boids3::computeLeaderAttraction(const Boid3 &boid)
{
    Vector3f toLeader = leaderPosition - boid.position;
    return toLeader.normalized();
    // return toLeader;
}

// Limit the speed of a vector to the maximum speed
Vector3f Boids3::limitSpeed(const Vector3f &velocity)
{
    if (velocity.norm() > maxSpeed)
    {
        return velocity.normalized() * maxSpeed;
    }

    if (velocity.norm() < 0.5)
    {
        // std::cout << "applyint minimun" << "\n";
        return velocity.normalized() * 0.5;
    }

    return velocity;
}

// Limit the acceleration
Vector3f Boids3::limitAccel(const Vector3f &accel)
{
    if (accel.norm() > maxAccel)
    {
        return accel.normalized() * maxAccel;
    }
    return accel;
}

// Repulsion force near surface
Vector3f Boids3::computeSurfaceRepulsion(const Boid3 &boid)
{

    if (boid.position.z() - 1.0 < 0.5)
    {
        // std::cout << "near surface limit, pushing down" << "\n";
        float repulsion = 1 / (boid.position.z() + __FLT_EPSILON__);
        return Vector3f(0, 0, repulsion);
    }
    else
    {
        return Vector3f::Zero();
    }
}

// Apply any external force to the whole grupo
void Boids3::AddGroupExternalForce(Vector3f force, double weight)
{
    groupExternalForce += force * weight;
}

void Boids3::updateBoids(double dt)
{
    // compute rules at current iteration
    for (auto &boid : boids)
    {
        Vector3f rules = computeRules(boid); // compute basic boid rules
        Vector3f leaderAttraction = computeLeaderAttraction(boid) * leaderAttractionWeight;
        Vector3f surfaceRepulsion = computeSurfaceRepulsion(boid) * 2; // why times 2

        boid.aceleration = (rules + leaderAttraction + surfaceRepulsion + groupExternalForce + boid.force) / mass;
        boid.aceleration = limitAccel(boid.aceleration);
    }

    // update boids for iteration+1
    spatial_grid.clear();
    for (auto &boid : boids)
    {
        boid.velocity += boid.aceleration * dt;
        boid.velocity = limitSpeed(boid.velocity);
        boid.position += boid.velocity * dt;
        boid.force = Vector3f::Zero(); // Reset force

        // Update spatial grid
        int cell_x = floor(boid.position.x() / cell_size);
        int cell_y = floor(boid.position.y() / cell_size);
        int cell_z = floor(boid.position.z() / cell_size);
        spatial_grid[hash(cell_x, cell_y, cell_z)].push_back(boid.id);
    }

    groupExternalForce = Vector3f::Zero();
}

Boids3::Boids3()
{
    // std::cout << "default boids constructor" << "\n";
}

Boids3::Boids3(std::string name, int numBoids, const Vector3f &initialLeaderPosition, int seed)
    : name(name), leaderPosition(initialLeaderPosition)
{
    std::random_device rd;
    std::mt19937 gen(seed);
    std::uniform_real_distribution<double> dis(-1.0, 1.0);

    for (u_int i = 0; i < numBoids; i++)
    {
        Boid3 boid(i);
        boid.position = Vector3f(dis(gen) + leaderPosition[0],
                                 dis(gen) + leaderPosition[1],
                                 dis(gen) + leaderPosition[2]);
        boids.push_back(boid);

        int cell_x = floor(boid.position.x() / (cell_size));
        int cell_y = floor(boid.position.y() / (cell_size));
        int cell_z = floor(boid.position.z() / (cell_size));
        spatial_grid[hash(cell_x, cell_y, cell_z)].push_back(i);
    }
}

int Boids3::hash(int x, int y, int z)
{
    return (x * 73856093) ^ (y * 19349663) ^ (z * 83492791);
}

std::vector<Boid3> &Boids3::getBoids()
{
    return boids;
}

void Boids3::printSettings()
{
    std::cout << "alignmentWeight: " << alignmentWeight << "\n"
              << "cohesionWeight: " << cohesionWeight << "\n"
              << "separationWeight: " << separationWeight << "\n"
              << "leaderAttractionWeight: " << leaderAttractionWeight << "\n"
              << "maxSpeed: " << maxSpeed << "\n"
              << "maxAccel: " << maxAccel << "\n"
              << "perceptionRadius: " << perceptionRadius << "\n"
              << "separationRadius: " << separationRadius << "\n"
              << "neighbor_cellsize: " << cell_size << "\n"
              << "mass: " << mass << "\n";
}

void Boids3::setLeaderPosition(const Vector3f &position)
{
    leaderPosition = position;
}

const Vector3f &Boids3::getLeaderPosition() const
{
    return leaderPosition;
}

const double &Boids3::getMass() const
{
    return mass;
}

void Boids3::setAlignmentWeight(const double weight)
{
    alignmentWeight = weight;
}

void Boids3::setCohesionWeight(const double weight)
{
    cohesionWeight = weight;
}

void Boids3::setSeparationWeight(const double weight)
{
    separationWeight = weight;
}

void Boids3::setLeaderAttractionWeight(const double weight)
{
    leaderAttractionWeight = weight;
}

void Boids3::setMaxSpeed(const double speed)
{
    maxSpeed = speed;
}

void Boids3::setMaxAccel(const double accel)
{
    maxAccel = accel;
}

void Boids3::setPerceptionRadius(const double radius)
{
    perceptionRadius = radius;
}

void Boids3::setSeparationRadius(const double radius)
{
    separationRadius = radius;
}

void Boids3::setNeighborCellSize(const double value)
{
    cell_size = value;
}

void Boids3::setMass(const double value)
{
    mass = value;
}
