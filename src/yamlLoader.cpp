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
    std::vector<std::string> container;
    std::vector<double> values;
    // create a stringstream.
    std::stringstream ss(str);
    while (ss.good())
    {
        std::string substr;
        getline(ss,substr,delimiter);
        // container.push_back(substr);
        values.push_back(std::stod(substr));
    }

    // int count = 0;
    assert(("mismatch between matrix size and no. of delimiters\n" ,(mat.rows()*mat.cols()) == values.size()));
    mat = Eigen::Map<Eigen::MatrixXd>(values.data(),mat.rows(),mat.cols());
    // for (int i = 0; i < mat.rows(); i++)
    // {
    //     for (int j = 0; j < mat.cols(); j++)
    //     {
    //         mat(i,j) = std::stod(container[count]);
    //         count += 1;
    //     }
        
    // }

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
