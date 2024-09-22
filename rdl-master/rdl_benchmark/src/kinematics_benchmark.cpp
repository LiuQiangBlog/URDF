// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// Copyright (c) 2011-2016 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#include <boost/program_options.hpp>

#include <rdl_benchmark/Timer.hpp>
#include <rdl_dynamics/Kinematics.hpp>
#include <rdl_dynamics/Model.hpp>
#include <rdl_urdfreader/urdfreader.hpp>

struct Samples
{
    std::vector<RobotDynamics::Math::VectorNd, Eigen::aligned_allocator<RobotDynamics::Math::VectorNd>> q_vec;
    std::vector<RobotDynamics::Math::VectorNd, Eigen::aligned_allocator<RobotDynamics::Math::VectorNd>> qdot_vec;
    std::vector<RobotDynamics::Math::VectorNd, Eigen::aligned_allocator<RobotDynamics::Math::VectorNd>> qddot_vec;
    std::vector<RobotDynamics::Math::VectorNd, Eigen::aligned_allocator<RobotDynamics::Math::VectorNd>> tau_vec;
};

Samples fillRandom(unsigned int q_size, unsigned int qdot_size, unsigned int num_samples)
{
    Samples samples;
    RobotDynamics::Math::VectorNd q = RobotDynamics::Math::VectorNd::Zero(q_size);
    RobotDynamics::Math::VectorNd qdot = RobotDynamics::Math::VectorNd::Zero(qdot_size);
    RobotDynamics::Math::VectorNd qddot = RobotDynamics::Math::VectorNd::Zero(qdot_size);
    RobotDynamics::Math::VectorNd tau = RobotDynamics::Math::VectorNd::Zero(qdot_size);

    for (unsigned int i = 0; i < num_samples; i++)
    {
        for (unsigned int j = 0; j < q_size; j++)
        {
            q[j] = (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) * 2. - 1.;
        }

        for (unsigned int j = 0; j < qdot_size; j++)
        {
            qdot[j] = (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) * 2. - 1.;
            qddot[j] = (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) * 2. - 1.;
            tau[j] = (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) * 2. - 1.;
        }

        samples.q_vec.push_back(q);
        samples.qdot_vec.push_back(qdot);
        samples.qddot_vec.push_back(qddot);
        samples.tau_vec.push_back(tau);
    }

    return samples;
}

int main(int argc, char* argv[])
{
    std::string urdf_file;
    unsigned int num_samples;

    boost::program_options::options_description description("Command line options kinematics benchmark");

    description.add_options()("help,h", "Display help options")("urdf", boost::program_options::value<std::string>(&urdf_file), "Path to the urdf")(
        "samples", boost::program_options::value<unsigned int>(&num_samples), "Number of benchmark samples to run");

    boost::program_options::variables_map command_line_args;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, description), command_line_args);
    boost::program_options::notify(command_line_args);

    if (command_line_args.count("help"))
    {
        std::cout << description << std::endl;
        return 0;
    }

    if (command_line_args.count("urdf"))
    {
        urdf_file = command_line_args["urdf"].as<std::string>();
    }

    num_samples = command_line_args.count("samples") ? command_line_args["samples"].as<unsigned int>() : 10000;  // default 10,000 samples

    RobotDynamics::ModelPtr model(new RobotDynamics::Model());
    if (!RobotDynamics::Urdf::urdfReadFromFile(urdf_file, model))
    {
        std::cerr << "Error reading urdf file" << std::endl;
        return 1;
    }

    Samples samples = fillRandom(model->q_size, model->qdot_size, num_samples);

    double duration = 0.;
    TimerInfo tinfo;
    timer_start(&tinfo);

    for (unsigned int i = 0; i < num_samples; i++)
    {
        RobotDynamics::updateKinematics(*model, samples.q_vec[i], samples.qdot_vec[i], samples.qddot_vec[i]);
    }

    duration = timer_stop(&tinfo);

    std::cout << "Kinematics for " << num_samples << " took " << duration << "s. Average calculation is " << duration / num_samples << std::endl;

    timer_start(&tinfo);

    for (unsigned int i = 0; i < num_samples; i++)
    {
        RobotDynamics::updateKinematicsParallel(*model, samples.q_vec[i], samples.qdot_vec[i], samples.qddot_vec[i]);
    }

    duration = timer_stop(&tinfo);

    std::cout << "Parallel kinematics for " << num_samples << " took " << duration << "s. Average calculation is " << duration / num_samples << std::endl;

    timer_start(&tinfo);

    for (unsigned int i = 0; i < num_samples; i++)
    {
        RobotDynamics::updateKinematicsCustom(*model, &samples.q_vec[i], &samples.qdot_vec[i], nullptr);
    }

    duration = timer_stop(&tinfo);

    std::cout << "Kinematics custom of only q/qdot for " << num_samples << " took " << duration << "s. Average calculation is " << duration / num_samples << std::endl;

    timer_start(&tinfo);

    for (unsigned int i = 0; i < num_samples; i++)
    {
        RobotDynamics::updateKinematicsCustomParallel(*model, &samples.q_vec[i], &samples.qdot_vec[i], &samples.qddot_vec[i]);
    }

    duration = timer_stop(&tinfo);

    std::cout << "Parallel kinematics custom of only q/qdot for " << num_samples << " took " << duration << "s. Average calculation is " << duration / num_samples
              << std::endl;

    return 0;
}
