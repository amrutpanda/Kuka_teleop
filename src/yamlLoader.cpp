#include "yamlLoader.hpp"

yamlLoader::yamlLoader(const std::string FilePath)
{
    mainNode = YAML::LoadFile(FilePath);
}

std::string yamlLoader::getWorldFilename() {
    return mainNode["files"]["world_file"].as<std::string>();
}

std::string yamlLoader::getRobotFilename() {
    return mainNode["files"]["robot_file"].as<std::string>();
}

float yamlLoader::getKp() {
    YAML::Node tnode = mainNode["robot"];
    return childNode["kp"].as<float>();
}

float yamlLoader::getKv() {
    YAML::Node tnode = mainNode["robot"];
    return tnode["kv"].as<float>();
}

Eigen::VectorXd yamlLoader::get_qinit() {
    std::cout << "I am here.\n";
    YAML::Node tnode = mainNode["robot"];
    Eigen::VectorXd vec;
    vec.resize(tnode.size());
    for (size_t i = 0; i < tnode.size(); i++)
    {
        std::cout << tnode[i] << "\n";
        vec[i] = tnode[i].as<float>();
    }
    return vec;
}

yamlLoader::~yamlLoader()
{
}



int main(int argc, char const *argv[])
{
    yamlLoader loader("../src/config.yaml");
    Eigen::VectorXd vec;
    vec = loader.get_qinit();
    return 0;
}
