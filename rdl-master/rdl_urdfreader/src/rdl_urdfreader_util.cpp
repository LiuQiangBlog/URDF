// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// Copyright (c) 2011-2016 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#include <rdl_dynamics/Model.hpp>
#include <rdl_dynamics/rdl_utils.hpp>

#include <rdl_urdfreader/urdfreader.hpp>

using namespace std;

bool verbose = false;
string filename = "";

void usage(const char* argv_0)
{
    cerr << "Usage: " << argv_0 << "[-v] [-m] [-d] <robot.urdf>" << endl;
    cerr << "  -v | --verbose            enable additional output" << endl;
    cerr << "  -d | --dof-overview       print an overview of the degress of freedom" << endl;
    cerr << "  -m | --model-hierarchy    print the hierarchy of the model" << endl;
    cerr << "  -h | --help               print this help" << endl;
    exit(1);
}

int main(int argc, char* argv[])
{
    if (argc < 2 || argc > 4)
    {
        usage(argv[0]);
    }

    bool verbose = false;
    bool dof_overview = false;
    bool model_hierarchy = false;

    string filename = argv[1];

    for (int i = 1; i < argc; ++i)
    {
        if (string(argv[i]) == "-v" || string(argv[i]) == "--verbose")
        {
            verbose = true;
        }
        else if (string(argv[i]) == "-d" || string(argv[i]) == "--dof-overview")
        {
            dof_overview = true;
        }
        else if (string(argv[i]) == "-m" || string(argv[i]) == "--model-hierarchy")
        {
            model_hierarchy = true;
        }
        else if (string(argv[i]) == "-h" || string(argv[i]) == "--help")
        {
            usage(argv[0]);
        }
        else
        {
            filename = argv[i];
        }
    }

    RobotDynamics::ModelPtr model(new RobotDynamics::Model());

    if (!RobotDynamics::Urdf::urdfReadFromFile(filename.c_str(), model, RobotDynamics::JointTypeFloatingBase, verbose))
    {
        cerr << "Loading of urdf model failed!" << endl;
        return -1;
    }

    cout << "Model loading successful!" << endl;

    if (dof_overview)
    {
        cout << "Degree of freedom overview:" << endl;
        cout << RobotDynamics::Utils::getModelDOFOverview(*model);
    }

    if (model_hierarchy)
    {
        cout << "Model Hierarchy:" << endl;
        cout << RobotDynamics::Utils::getModelHierarchy(*model);
    }

    return 0;
}
