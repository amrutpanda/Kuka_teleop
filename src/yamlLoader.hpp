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
public:
    yamlLoader(const std::string FilePath );
    std::string getWorldFilename();
    std::string getRobotFilename();
    float getKp();
    float getKv();
    Eigen::VectorXd get_qinit(int matRows, int matCols);
    ~yamlLoader();
};

