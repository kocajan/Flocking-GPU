#include <string>

#include "app/Application.hpp"


int main(int argc, char** argv) {
    std::string configPath = "cfg";

    if (argc > 1) {
        configPath = argv[1];
    }

    Application app;
    app.run(configPath);
    return 0;
}
