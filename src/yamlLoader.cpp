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
    return tnode["kp"].as<float>();
}

float yamlLoader::getKv() {
    YAML::Node tnode = mainNode["robot"];
    return tnode["kv"].as<float>();
}

Eigen::VectorXd yamlLoader::get_qlower() {
    std::string exp = mainNode["robot"]["q_lower"].as<std::string>();
    std::vector<double> values;
    // create a stringstream.
    std::stringstream ss(exp);
    while (ss.good())
    {
        std::string substr;
        getline(ss,substr,',');
        values.push_back(std::stod(substr));

    }
    Eigen::VectorXd v(Eigen::Map<Eigen::VectorXd>(values.data(),values.size()));
    return v;
      
}

Eigen::VectorXd yamlLoader::get_qupper() {
    std::string exp = mainNode["robot"]["q_upper"].as<std::string>();
    std::vector<double> values;
    // create a stringstream.
    std::stringstream ss(exp);
    while (ss.good())
    {
        std::string substr;
        getline(ss,substr,',');
        values.push_back(std::stod(substr));

    }
    Eigen::VectorXd v(Eigen::Map<Eigen::VectorXd>(values.data(),values.size()));
    return v;
      
}

Eigen::VectorXd yamlLoader::get_qinit() {
    std::string exp = mainNode["robot"]["q_init"].as<std::string>();
    std::vector<double> values;
    // create a stringstream.
    std::stringstream ss(exp);
    while (ss.good())
    {
        std::string substr;
        getline(ss,substr,',');
        values.push_back(std::stod(substr));

    }
    Eigen::VectorXd v(Eigen::Map<Eigen::VectorXd>(values.data(),values.size()));
    return v;
      
}

void yamlLoader::strToEigen(std::string& str, char delimiter, Eigen::MatrixXd& mat ) {
    std::vector<double> values;
    // create a stringstream.
    std::stringstream ss(str);
    while (ss.good())
    {
        std::string substr;
        getline(ss,substr,delimiter);
        values.push_back(std::stod(substr));
    }

    assert(("mismatch between matrix size and no. of delimiters\n" ,(mat.rows()*mat.cols()) == values.size()));
    mat = Eigen::Map<Eigen::MatrixXd>(values.data(),mat.rows(),mat.cols());

}

void yamlLoader::strToEigenVector(std::string& str, char delimiter ,Eigen::VectorXd& v) {
    std::vector<double> values;
    // create a stringstream.
    std::stringstream ss(str);
    while (ss.good())
    {
        std::string substr;
        getline(ss,substr,',');
        values.push_back(std::stod(substr));

    }
    // Eigen::VectorXd v(Eigen::Map<Eigen::VectorXd>(values.data(),values.size()));
    v = Eigen::Map<Eigen::VectorXd>(values.data(),values.size());
}

yamlLoader::~yamlLoader()
{

}



// int main(int argc, char const *argv[])
// {
//     yamlLoader loader("../src/config.yaml");
//     Eigen::VectorXd vec;
//     std::cout << loader.getRobotFilename() << "\n";
//     vec = loader.get_qinit();
//     std::cout << vec << "\n";

//     return 0;
// }
