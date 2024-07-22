#include <iostream>
#include <string>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <yaml-cpp/depthguard.h>
#include <yaml-cpp/yaml.h>

class yamlLoader
{
private:
    YAML::Node mainNode;
    YAML::Node childNode;
public:
    yamlLoader(const std::string FilePath );
    std::string getWorldFilename();
    std::string getRobotFilename();
    float getKp();
    float getKv();
    Eigen::VectorXd get_qinit();
    ~yamlLoader();
};

