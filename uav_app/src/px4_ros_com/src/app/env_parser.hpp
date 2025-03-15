#include <unordered_map>
#include <curl/curl.h>
#include <iostream>
#include <fstream>
#include <string>


class EnvParser { 
    public:
        EnvParser();
        void load_env(std::string fp);
        uint32_t parse_env(); 
        std::string get_var(std::string key);

    private:
        std::string filepath;
        std::unordered_map<std::string, std::string> env_vars;
};

EnvParser::EnvParser() {}

void EnvParser::load_env(std::string fp) {
    filepath = fp;
}

uint32_t EnvParser::parse_env() {
    env_vars.clear();

    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "file is not open" << std::endl;
        return 0;
    }

    std::string env_line;

    while (std::getline(file, env_line)) {
        if (env_line.empty()) 
            break;

        size_t pos = env_line.find('=');

        std::string key = env_line.substr(pos + 1);
        std::string var = env_line.substr(0, pos);

        env_vars[key] = var;
    }
    
    file.close();

    return 1;
}

std::string EnvParser::get_var(std::string key) {
    return env_vars[key];
}

 

