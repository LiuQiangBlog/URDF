// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
//
// SPDX-License-Identifier: GPL-3.0-only

//#include <Eigen/Dense>
//
//#include <KDLRobot.hpp>
//#include <iostream>
//#include <string>
//
//int main(int argc, char* argv[]) {
//    // Check the program call.
//
//    std::string urdf_file = std::string("D:\\wbc\\models\\kuka\\urdf\\kuka_iiwa.urdf");
//
//    // Create a KDL Robot object, with a kinematic chain starting at "panda_link0" and ending at "panda_tip"
//    KDLRobot kdl_robot(urdf_file, "kuka_lbr_l_link_0", "kuka_lbr_l_tcp", Eigen::VectorXd::Zero(7), Eigen::VectorXd::Zero(7));
//
//    kdl_robot.printChain();
//
//    // Print some values of interest
//    std::cout << "------" << std::endl;
//    std::cout << "Joint positions: " << kdl_robot.getJointsPos() << std::endl;
//    std::cout << "------" << std::endl;
//    std::cout << "Joint velocities: " << kdl_robot.getJointsVel() << std::endl;
//    std::cout << "------" << std::endl;
//    std::cout << "EE Pos: \n" << kdl_robot.getEEPosition() << std::endl;
//    std::cout << "------" << std::endl;
//    std::cout << "EE Vel: \n" << kdl_robot.getEEVelocity() << std::endl;
//    std::cout << "------" << std::endl;
//    std::cout << "EE Orn: \n" << kdl_robot.getEEOrnQuat() << std::endl;
//    std::cout << "------" << std::endl;
//    std::cout << "J: \n" << kdl_robot.J() << std::endl;
//
//    return 0;
//}

#include <chain.hpp>
#include <chainidsolver.hpp>
#include <chainidsolver_recursive_newton_euler.hpp>
#include <joint.hpp>
#include <segment.hpp>
#include <KDLRobot.hpp>
#include <iostream>

using namespace KDL;
int main(int argc, char** argv)
{
    std::string urdf_file = std::string("D:\\wbc\\models\\kuka\\urdf\\kuka_iiwa.urdf");
    // �� URDF �ļ��н���������ģ��
    KDLRobot kdl_robot(urdf_file, "kuka_lbr_l_link_0", "kuka_lbr_l_tcp", Eigen::VectorXd::Zero(7), Eigen::VectorXd::Zero(7));
    auto &chain = kdl_robot.chain_;
    // �����涯��ѧ�����
    KDL::ChainIdSolver_RNE idSolver(chain, Vector(0,0,-9.81));

    // ��������
    KDL::JntArray q(chain.getNrOfJoints()); // �ؽ�λ��
    KDL::JntArray qdot(chain.getNrOfJoints()); // �ؽ��ٶ�
    KDL::JntArray qddot(chain.getNrOfJoints()); // �ؽڼ��ٶ�
    KDL::JntArray torque(chain.getNrOfJoints()); // �ؽ�����

    double q1[7] = {1.18169546047932,0.322295169295169,-0.322295169295169,0.322295169295169,0.00564799672565738,0.322295169295169,0.00564799672565738};
    double qd1[7] = {0.1745,0.1745,-0.1745,0.1745,0.1745,0.1745,0.1745};
    double qdd1[7] = {0.1,0.1,-0.1,0.1,0.1,0.1,0.1};
    // �������������
    for (unsigned int i = 0; i < chain.getNrOfJoints(); ++i)
    {
        q(i) = q1[i]; // λ��
        qdot(i) = qd1[i]; // �ٶ�
        qddot(i) = qdd1[i]; // ���ٶ�
    }

    // �����涯��ѧ
    Wrenches f_ext;
    if (idSolver.CartToJnt(q, qdot, qddot, f_ext, torque) < 0)
    {
        std::cerr << "Failed to compute inverse dynamics" << std::endl;
        return -1;
    }

    // ����ؽ�����
    for (unsigned int i = 0; i < chain.getNrOfJoints(); ++i)
    {
        std::cout << "Joint " << i << " Torque: " << torque(i) << std::endl;
    }

    return 0;
}
