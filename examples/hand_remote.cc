/*!
    \file examples/hand_remote.cc
    \brief Control manipulator example

    \author João Borrego : jsbruglie
*/

#include "hand_remote.hh"

int main(int _argc, char **_argv){

    // Command-line arguments
    std::string config_file, robot;
    parseArgs(_argc, _argv, config_file, robot);

    // Load gazebo as a client
    gazebo::client::setup(_argc, _argv);

    // Interface
    Interface api;
    // Init interface with config file
    if (!api.init(config_file, robot)){
        std::cout << "Exiting..." << std::endl;
        exit(EXIT_FAILURE);
    }

    // Main loop - Keyboard input
    api.loop();

    // Shut down
    gazebo::client::shutdown();
    return 0;
}


//////////////////////////////////////////////////
void parseArgs(int argc, char** argv, std::string & cfg_dir, std::string & robot){
    int opt;
    bool c, r;

    while ((opt = getopt(argc,argv,"c: r:")) != EOF){
        switch (opt){
            case 'c':
                c = true;
		cfg_dir = optarg;
		break;

            case 'r':
                r = true;
		robot = optarg;
		break;

            case '?':
                std::cerr << "usage:   " + std::string(argv[0]) + " [options]\noptions: -c <robot configuration yaml>\n         -r <robot>\n";

            default:
                std::cout << std::endl;
                exit(EXIT_FAILURE);
        }
    }

    if (!c || !r){
        std::cerr << "usage:   " + std::string(argv[0]) + " [options]\noptions: -c <robot configuration yaml>\n         -r <robot>\n";
        exit(EXIT_FAILURE);
    }

    std::cout << "Parameters:\n" << "   Robot configuration yaml '" << cfg_dir << "'\n" << "   Robot                    '" << robot << "'\n" << std::endl;
}

