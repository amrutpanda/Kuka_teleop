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
    void strToEigen(std::string& str, char delimiter ,Eigen::MatrixXd& mat);
    void strToEigenVector(std::string& str, char delimiter ,Eigen::VectorXd& v);
public:
    yamlLoader(const std::string FilePath );
    std::string getWorldFilename();
    std::string getRobotFilename();
    float getKp();
    float getKv();
    Eigen::VectorXd get_qinit();
    Eigen::MatrixXd get_rinit();
    ~yamlLoader();
};

