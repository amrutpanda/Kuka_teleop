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

Eigen::VectorXd yamlLoader::get_qinit(int matRows = 0 , int matCols = 1) {
    // matRows 0 means the matrix is vector only.
    std::string exp = mainNode["robot"]["q_init"].as<std::string>();
    // parse the string and store it inside a vector for further processing.
    std::vector<std::string> container;
    // create a stringstream.
    std::stringstream ss(exp);
    while (ss.good())
    {
        std::string substr;
        getline(ss,substr,',');
        container.push_back(substr);
    }

    Eigen::VectorXd v(container.size());
    for (int i = 0; i < container.size(); i++)
    {
        v(i) = std::stod(container[i]);
    }
    return v;
      
}

void yamlLoader::strToEigen(std::string& str, char delimiter, Eigen::MatrixXd& mat ) {
    std::vector<std::string> container;
    // create a stringstream.
    std::stringstream ss(str);
    while (ss.good())
    {
        std::string substr;
        getline(ss,substr,delimiter);
        container.push_back(substr);
    }

    int count = 0;
    assert(("mismatch between matrix size and no. of delimiters\n" ,(mat.rows()*mat.cols()) == container.size()));
    for (int i = 0; i < mat.rows(); i++)
    {
        for (int j = 0; j < mat.cols(); j++)
        {
            mat(i,j) = std::stod(container[count]);
            count += 1;
        }
        
    }

}

yamlLoader::~yamlLoader()
{
}



int main(int argc, char const *argv[])
{
    yamlLoader loader("../src/config.yaml");
    Eigen::VectorXd vec;
    std::cout << loader.getRobotFilename() << "\n";
    vec = loader.get_qinit();
    std::cout << vec << "\n";

    return 0;
}
