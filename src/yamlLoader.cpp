#include "yamlLoader.hpp"

yamlLoader::yamlLoader()
{

}

yamlLoader::~yamlLoader()
{
}

void yamlLoader::loadFile(std::string filename) {

    try
    {
        std::ifstream input_file_stream;
        input_file_stream.open(filename,std::ios::in);
        std::stringstream ss;
        ss << input_file_stream.rdbuf();
        file_string = ss.str();
        // load to mainNode.
        mainNode = YAML::LoadFile(filename);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        std::runtime_error("Error while reading file: " + filename);
    }
    
    
}

void yamlLoader::get_data_from_param(std::string param_string, std::string& strval) {

    try
    {
        // split the param_string.
        std::vector<std::string> container;
        std::stringstream ss;
        std::string substr;
        ss << param_string;

        while (ss.good())
        {
            getline(ss,substr,':');
            container.push_back(substr);
        }
        
        YAML::Node tnode,snode;
        // tnode = mainNode;
        tnode = YAML::Load(file_string);
        // don't know why reading from file_stream does not work.
        // tnode = YAML::LoadFile(Filename);
        for (int i = 0; i < container.size(); i++)
        {
            snode = tnode[container[i]]; 
            tnode = snode;
        }
        strval = tnode.as<std::string>();
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        std::runtime_error("Error while reading the values of param: " + param_string);
    }
    
}


std::string yamlLoader::GetParamStr(std::string param_name) {
    std::string str;
    get_data_from_param(param_name,str);
    return str;
}

int yamlLoader::GetParamInt(std::string param_name) {
    std::string str;
    get_data_from_param(param_name,str);
    return std::stoi(str);
}

double yamlLoader::GetParamDouble(std::string param_name) {
    std::string str;
    get_data_from_param(param_name,str);
    return std::stod(str);
}

void yamlLoader::GetParamEigenVector(std::string param_name, Eigen::VectorXd& v) {
    std::string str;
    std::string substr;
    get_data_from_param(param_name,str);
    std::vector<double> value;
    std::stringstream ss;
    ss << str;
    while (ss.good())
    {
        getline(ss,substr,',');
        value.push_back(std::stod(substr));
    }
    try
    {
        v = Eigen::Map<Eigen::VectorXd>(value.data(),value.size());      
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        std::runtime_error("Error while mapping to Eigen vector from param : " + param_name);
    }
    
}

void yamlLoader::GetParamEigenMatrix(std::string param_name, Eigen::MatrixXd& mat ) {
    std::string str;
    std::string substr;
    get_data_from_param(param_name,str);
    std::vector<double> value;
    std::stringstream ss;
    ss << str;
    while (ss.good())
    {
        getline(ss,substr,',');
        value.push_back(std::stod(substr));
    }

    try
    {
        mat = Eigen::Map<Eigen::MatrixXd>(value.data(),mat.rows(), mat.cols());
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        std::runtime_error("Error while mapping to Eigen matrix from param : " + param_name);
    }
    
}