#include <vector>
#include <string>
#include <iostream>
#include <Eigen/Core>

struct ModelData
{
    enum ModelType
    {
        MESH,
        SURFACE
    };

    std::string name;
    ModelType type;
    // std::vector<double> translation;
    // std::vector<double> rotation;
    Eigen::Vector3f translation;
    Eigen::Vector3f rotation;
    double scale;

    // surface
    std::vector<Eigen::Vector3f> points;
    std::vector<Eigen::Vector3d> normals;

    void print()
    {
        std::cout << "Model name: " << name << "\n";
        std::cout << "Model size: " << scale << "\n";
        std::cout << "Model X: " << translation[0]
                  << ", Y: " << translation[1]
                  << ", Z: " << translation[2]
                  << "\n\n";
    }
};

class ObjectDetailsParser
{
public:
    ObjectDetailsParser(std::string scenario_path, std::string models_dir);

    std::vector<ModelData> iterateSceneModels(const std::string &scenario_path, const std::string &models_dir);
    void loadModelGetSize(std::string &model_path);
    void getSurfaceNormals(std::string &model_path, ModelData &mdata);
    Eigen::Vector3f stringToVector(std::string text_coords);

private:
    std::vector<ModelData> modelsData;
    std::string scenario_path_;
    std::string models_dir_;
};