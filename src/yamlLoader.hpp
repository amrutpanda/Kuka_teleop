#include <iostream>
#include <fstream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

#include <yaml-cpp/yaml.h>

class yamlLoader
{
private:
    std::string file_string;
    YAML::Node mainNode;
    void get_data_from_param(std::string param_string, std::string& strval);
public:
    yamlLoader();
    void loadFile(std::string filename);
    std::string GetParamStr(std::string param_name);
    int GetParamInt(std::string param_name);
    double GetParamDouble(std::string param_name);
    void GetParamEigenVector(std::string param_name, Eigen::VectorXd& v);
    void GetParamEigenMatrix(std::string param_name, Eigen::MatrixXd& mat);
    ~yamlLoader();
};

