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
    Eigen::VectorXd getKpV();
    Eigen::VectorXd getKvV();
    float getKv();
    Eigen::VectorXd get_qlower();
    Eigen::VectorXd get_qupper();
    Eigen::VectorXd get_qinit();
    Eigen::MatrixXd get_rinit();

    // new functions
    std::string GetParamStr(std::string param_name);
    int GetParamInt(std::string param_name);
    double GetParamDouble(std::string param_name);
    Eigen::VectorXd GetParamVector(std::string param_name);
    Eigen::MatrixXd GetParamMatrix(std::string param_name);

    ~yamlLoader();
};

