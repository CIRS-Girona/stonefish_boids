#include <stonefish_boids/enviroment/ScnParser.h>

#include <iostream>
#include <fstream>
#include <open3d/Open3D.h>
#include <open3d/geometry/TriangleMesh.h>
#include <rapidxml/rapidxml.hpp>
#include <rapidxml/rapidxml_utils.hpp>

ObjectDetailsParser::ObjectDetailsParser(std::string scenario_path, std::string models_dir)
    : scenario_path_(scenario_path), models_dir_(models_dir)
{
    std::cout << "ScnParser constructor" << "\n";
    std::cout << scenario_path_ << "\n";
    std::cout << models_dir_ << "\n";
}

std::vector<ModelData> ObjectDetailsParser::iterateSceneModels(const std::string &scenario_path, const std::string &models_dir)
{
    std::vector<ModelData> models_data;

    rapidxml::file<> xml_file(scenario_path.c_str());
    rapidxml::xml_document<> xml_doc;
    xml_doc.parse<0>(xml_file.data());

    rapidxml::xml_node<> *scenario_node = xml_doc.first_node("scenario");
    rapidxml::xml_attribute<> *obj_path;
    rapidxml::xml_attribute<> *obj_scale;
    rapidxml::xml_attribute<> *obj_trans;
    rapidxml::xml_attribute<> *obj_rot;
    for (
        rapidxml::xml_node<> *static_node = scenario_node->first_node("static");
        static_node;
        static_node = static_node->next_sibling())
    {
        ModelData mdata;
        mdata.name = static_node->first_attribute("name")->value();

        std::string typestr = static_node->first_attribute("type")->value();

        if (typestr.compare("model"))
        {
            std::cout << "skipping: " << mdata.name << "\n";
            continue;
        }

        mdata.type = ModelData::ModelType::MESH;
        obj_path = static_node->first_node("visual")->first_node("mesh")->first_attribute("filename");

        auto model_path = models_dir + obj_path->value();
        ObjectDetailsParser::loadModelGetSize(model_path);

        obj_scale = static_node->first_node("visual")->first_node("mesh")->first_attribute("scale");
        mdata.scale = atof(obj_scale->value());

        obj_trans = static_node->first_node("world_transform")->first_attribute("xyz");
        mdata.translation = stringToVector(obj_trans->value());

        obj_rot = static_node->first_node("world_transform")->first_attribute("rpy");
        mdata.rotation = stringToVector(obj_rot->value());

        if (mdata.name == "Seabed")
        {
            std::cout << "seabed" << "\n";
            mdata.type = ModelData::ModelType::SURFACE;
            ObjectDetailsParser::getSurfaceNormals(model_path, mdata);
        }

        models_data.push_back(mdata);
    };
    return models_data;
};

void ObjectDetailsParser::loadModelGetSize(std::string &model_path)
{
    open3d::geometry::TriangleMesh model_mesh;
    if (open3d::io::ReadTriangleMesh(model_path, model_mesh))
    {
        auto bounding_box = model_mesh.GetAxisAlignedBoundingBox();
        std::cout << "Bounds: " << bounding_box.GetMaxBound() << "\n";
    }
    else
    {
        std::cerr << ("could not load:" + model_path);
    }
};

void ObjectDetailsParser::getSurfaceNormals(std::string &model_path, ModelData &mdata)
{
    open3d::geometry::TriangleMesh model_mesh;

    Eigen::AngleAxisd rollAngle(mdata.rotation[0], Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(mdata.rotation[1], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(mdata.rotation[2], Eigen::Vector3d::UnitZ());

    Eigen::Quaternion<double> q = yawAngle * pitchAngle * rollAngle;
    Eigen::Matrix3d rotationMatrix = q.matrix();

    // model_mesh.SamplePointsUniformly(500, true);

    if (open3d::io::ReadTriangleMesh(model_path, model_mesh))
    {
        model_mesh.Rotate(rotationMatrix, Eigen::Vector3d::Zero());
        model_mesh = *model_mesh.SubdivideMidpoint(2);
        model_mesh.ComputeTriangleNormals();
        mdata.normals = model_mesh.triangle_normals_;

        for (auto &tri : model_mesh.triangles_)
        {
            // Get actual vertex positions
            Eigen::Vector3d v1 = model_mesh.vertices_[tri(0)];
            Eigen::Vector3d v2 = model_mesh.vertices_[tri(1)];
            Eigen::Vector3d v3 = model_mesh.vertices_[tri(2)];

            // Compute centroid
            Eigen::Vector3f centroid = ((v1 + v2 + v3) / 3.0).cast<float>();
            centroid = centroid + mdata.translation;
            mdata.points.push_back(centroid);
        }
    }
    else
    {
        std::cerr << ("could not load:" + model_path);
    }
};

Eigen::Vector3f ObjectDetailsParser::stringToVector(std::string text_coords)
{
    Eigen::Vector3f vec;
    std::stringstream string_stream(text_coords);
    std::string string_temp;
    int count = 0;
    while (getline(string_stream, string_temp, ' '))
    {
        vec[count] = stof(string_temp);
        count++;
    }
    return vec;
};

// int main()
// {
//     string scenario_path = "/home/jad/Documents/codes/cpp-parser/scenes/basic1.scn";
//     string models_dir = "/home/jad/stonefish_ws/src/estonefish-scenes/data/";

//     ObjectDetailsParser objects_details_parser(
//         scenario_path,
//         models_dir);
//     objects_details_parser.iterateSceneModels(
//         scenario_path,
//         models_dir);
//     return 0;
// }
