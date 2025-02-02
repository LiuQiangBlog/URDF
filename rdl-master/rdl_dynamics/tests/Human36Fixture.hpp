// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// Copyright (c) 2011-2016 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#ifndef __RDL_HUMAN36_FIXTURE_H__
#define __RDL_HUMAN36_FIXTURE_H__

#include "UnitTestUtils.hpp"
#include "rdl_dynamics/Model.hpp"
#include "rdl_dynamics/Contacts.hpp"
#include <gtest/gtest.h>

struct Human36 : public testing::Test
{
    double fluid_density = 1000.0;
    RobotDynamics::Model::SharedPtr model;
    RobotDynamics::Model::SharedPtr model_emulated;
    RobotDynamics::Model::SharedPtr model_3dof;

    RobotDynamics::Math::VectorNd q;
    RobotDynamics::Math::VectorNd qdot;
    RobotDynamics::Math::VectorNd qddot;
    RobotDynamics::Math::VectorNd tau;

    RobotDynamics::Math::VectorNd qddot_emulated;
    RobotDynamics::Math::VectorNd qddot_3dof;

    RobotDynamics::ConstraintSet constraints_1B1C_emulated;
    RobotDynamics::ConstraintSet constraints_1B4C_emulated;
    RobotDynamics::ConstraintSet constraints_4B4C_emulated;

    RobotDynamics::ConstraintSet constraints_1B1C_3dof;
    RobotDynamics::ConstraintSet constraints_1B4C_3dof;
    RobotDynamics::ConstraintSet constraints_4B4C_3dof;

    enum SegmentName
    {
        SegmentPelvis = 0,
        SegmentThigh,
        SegmentShank,
        SegmentFoot,
        SegmentMiddleTrunk,
        SegmentUpperTrunk,
        SegmentUpperArm,
        SegmentLowerArm,
        SegmentHand,
        SegmentHead,
        SegmentNameLast
    };

    enum BodyName
    {
        BodyPelvis,
        BodyThighRight,
        BodyShankRight,
        BodyFootRight,
        BodyThighLeft,
        BodyShankLeft,
        BodyFootLeft,
        BodyMiddleTrunk,
        BodyUpperTrunk,
        BodyUpperArmRight,
        BodyLowerArmRight,
        BodyHandRight,
        BodyUpperArmLeft,
        BodyLowerArmLeft,
        BodyHandLeft,
        BodyHead,
        BodyNameLast
    };

    double SegmentLengths[SegmentNameLast];
    double SegmentMass[SegmentNameLast];
    double SegmentVolume[SegmentNameLast];
    double SegmentCOM[SegmentNameLast][3];
    double SegmentCOB[SegmentNameLast][3];
    double SegmentRadiiOfGyration[SegmentNameLast][3];

    unsigned int body_id_emulated[BodyNameLast];
    unsigned int body_id_3dof[BodyNameLast];

    void initParameters()
    {
        SegmentLengths[SegmentPelvis] = 0.1457;
        SegmentLengths[SegmentThigh] = 0.4222;
        SegmentLengths[SegmentShank] = 0.4403;
        SegmentLengths[SegmentFoot] = 0.1037;
        SegmentLengths[SegmentMiddleTrunk] = 0.2155;
        SegmentLengths[SegmentUpperTrunk] = 0.2421;
        SegmentLengths[SegmentUpperArm] = 0.2817;
        SegmentLengths[SegmentLowerArm] = 0.2689;
        SegmentLengths[SegmentHand] = 0.0862;
        SegmentLengths[SegmentHead] = 0.2429;

        SegmentMass[SegmentPelvis] = 0.8154;
        SegmentMass[SegmentThigh] = 10.3368;
        SegmentMass[SegmentShank] = 3.1609;
        SegmentMass[SegmentFoot] = 1.001;
        SegmentMass[SegmentMiddleTrunk] = 16.33;
        SegmentMass[SegmentUpperTrunk] = 15.96;
        SegmentMass[SegmentUpperArm] = 1.9783;
        SegmentMass[SegmentLowerArm] = 1.1826;
        SegmentMass[SegmentHand] = 0.4453;
        SegmentMass[SegmentHead] = 5.0662;

        SegmentCOM[SegmentPelvis][0] = 0.;
        SegmentCOM[SegmentPelvis][1] = 0.;
        SegmentCOM[SegmentPelvis][2] = 0.0891;

        SegmentCOM[SegmentThigh][0] = 0.;
        SegmentCOM[SegmentThigh][1] = 0.;
        SegmentCOM[SegmentThigh][2] = -0.1729;

        SegmentCOM[SegmentShank][0] = 0.;
        SegmentCOM[SegmentShank][1] = 0.;
        SegmentCOM[SegmentShank][2] = -0.1963;

        SegmentCOM[SegmentFoot][0] = 0.1254;
        SegmentCOM[SegmentFoot][1] = 0.;
        SegmentCOM[SegmentFoot][2] = -0.0516;

        SegmentCOM[SegmentMiddleTrunk][0] = 0.;
        SegmentCOM[SegmentMiddleTrunk][1] = 0.;
        SegmentCOM[SegmentMiddleTrunk][2] = 0.1185;

        SegmentCOM[SegmentUpperTrunk][0] = 0.;
        SegmentCOM[SegmentUpperTrunk][1] = 0.;
        SegmentCOM[SegmentUpperTrunk][2] = 0.1195;

        SegmentCOM[SegmentUpperArm][0] = 0.;
        SegmentCOM[SegmentUpperArm][1] = 0.;
        SegmentCOM[SegmentUpperArm][2] = -0.1626;

        SegmentCOM[SegmentLowerArm][0] = 0.;
        SegmentCOM[SegmentLowerArm][1] = 0.;
        SegmentCOM[SegmentLowerArm][2] = -0.1230;

        SegmentCOM[SegmentHand][0] = 0.;
        SegmentCOM[SegmentHand][1] = 0.;
        SegmentCOM[SegmentHand][2] = -0.0680;

        SegmentCOM[SegmentHead][0] = 0.;
        SegmentCOM[SegmentHead][1] = 0.;
        SegmentCOM[SegmentHead][2] = 1.1214;

        SegmentVolume[SegmentPelvis] = SegmentMass[SegmentPelvis] / fluid_density;
        SegmentVolume[SegmentThigh] = SegmentMass[SegmentThigh] / fluid_density;
        SegmentVolume[SegmentShank] = SegmentMass[SegmentShank] / fluid_density;
        SegmentVolume[SegmentFoot] = SegmentMass[SegmentFoot] / fluid_density;
        SegmentVolume[SegmentMiddleTrunk] = SegmentMass[SegmentMiddleTrunk] / fluid_density;
        SegmentVolume[SegmentUpperTrunk] = SegmentMass[SegmentUpperTrunk] / fluid_density;
        SegmentVolume[SegmentUpperArm] = SegmentMass[SegmentUpperArm] / fluid_density;
        SegmentVolume[SegmentLowerArm] = SegmentMass[SegmentLowerArm] / fluid_density;
        SegmentVolume[SegmentHand] = SegmentMass[SegmentHand] / fluid_density;
        SegmentVolume[SegmentHead] = SegmentMass[SegmentHead] / fluid_density;

        SegmentCOB[SegmentPelvis][0] = SegmentCOM[SegmentPelvis][0];
        SegmentCOB[SegmentPelvis][1] = SegmentCOM[SegmentPelvis][1];
        SegmentCOB[SegmentPelvis][2] = SegmentCOM[SegmentPelvis][2];

        SegmentCOB[SegmentThigh][0] = SegmentCOM[SegmentThigh][0];
        SegmentCOB[SegmentThigh][1] = SegmentCOM[SegmentThigh][1];
        SegmentCOB[SegmentThigh][2] = SegmentCOM[SegmentThigh][2];

        SegmentCOB[SegmentShank][0] = SegmentCOM[SegmentShank][0];
        SegmentCOB[SegmentShank][1] = SegmentCOM[SegmentShank][1];
        SegmentCOB[SegmentShank][2] = SegmentCOM[SegmentShank][2];

        SegmentCOB[SegmentFoot][0] = SegmentCOM[SegmentFoot][0];
        SegmentCOB[SegmentFoot][1] = SegmentCOM[SegmentFoot][1];
        SegmentCOB[SegmentFoot][2] = SegmentCOM[SegmentFoot][2];

        SegmentCOB[SegmentMiddleTrunk][0] = SegmentCOM[SegmentMiddleTrunk][0];
        SegmentCOB[SegmentMiddleTrunk][1] = SegmentCOM[SegmentMiddleTrunk][1];
        SegmentCOB[SegmentMiddleTrunk][2] = SegmentCOM[SegmentMiddleTrunk][2];

        SegmentCOB[SegmentUpperTrunk][0] = SegmentCOM[SegmentUpperTrunk][0];
        SegmentCOB[SegmentUpperTrunk][1] = SegmentCOM[SegmentUpperTrunk][1];
        SegmentCOB[SegmentUpperTrunk][2] = SegmentCOM[SegmentUpperTrunk][2];

        SegmentCOB[SegmentUpperArm][0] = SegmentCOM[SegmentUpperArm][0];
        SegmentCOB[SegmentUpperArm][1] = SegmentCOM[SegmentUpperArm][1];
        SegmentCOB[SegmentUpperArm][2] = SegmentCOM[SegmentUpperArm][2];

        SegmentCOB[SegmentLowerArm][0] = SegmentCOM[SegmentLowerArm][0];
        SegmentCOB[SegmentLowerArm][1] = SegmentCOM[SegmentLowerArm][1];
        SegmentCOB[SegmentLowerArm][2] = SegmentCOM[SegmentLowerArm][2];

        SegmentCOB[SegmentHand][0] = SegmentCOM[SegmentHand][0];
        SegmentCOB[SegmentHand][1] = SegmentCOM[SegmentHand][1];
        SegmentCOB[SegmentHand][2] = SegmentCOM[SegmentHand][2];

        SegmentCOB[SegmentHead][0] = SegmentCOM[SegmentHead][0];
        SegmentCOB[SegmentHead][1] = SegmentCOM[SegmentHead][1];
        SegmentCOB[SegmentHead][2] = SegmentCOM[SegmentHead][2];

        SegmentRadiiOfGyration[SegmentPelvis][0] = 0.0897;
        SegmentRadiiOfGyration[SegmentPelvis][1] = 0.0855;
        SegmentRadiiOfGyration[SegmentPelvis][2] = 0.0803;

        SegmentRadiiOfGyration[SegmentThigh][0] = 0.1389;
        SegmentRadiiOfGyration[SegmentThigh][1] = 0.0629;
        SegmentRadiiOfGyration[SegmentThigh][2] = 0.1389;

        SegmentRadiiOfGyration[SegmentShank][0] = 0.1123;
        SegmentRadiiOfGyration[SegmentShank][1] = 0.0454;
        SegmentRadiiOfGyration[SegmentShank][2] = 0.1096;

        SegmentRadiiOfGyration[SegmentFoot][0] = 0.0267;
        SegmentRadiiOfGyration[SegmentFoot][1] = 0.0129;
        SegmentRadiiOfGyration[SegmentFoot][2] = 0.0254;

        SegmentRadiiOfGyration[SegmentMiddleTrunk][0] = 0.0970;
        SegmentRadiiOfGyration[SegmentMiddleTrunk][1] = 0.1009;
        SegmentRadiiOfGyration[SegmentMiddleTrunk][2] = 0.0825;

        SegmentRadiiOfGyration[SegmentUpperTrunk][0] = 0.1273;
        SegmentRadiiOfGyration[SegmentUpperTrunk][1] = 0.1172;
        SegmentRadiiOfGyration[SegmentUpperTrunk][2] = 0.0807;

        SegmentRadiiOfGyration[SegmentUpperArm][0] = 0.0803;
        SegmentRadiiOfGyration[SegmentUpperArm][1] = 0.0758;
        SegmentRadiiOfGyration[SegmentUpperArm][2] = 0.0445;

        SegmentRadiiOfGyration[SegmentLowerArm][0] = 0.0742;
        SegmentRadiiOfGyration[SegmentLowerArm][1] = 0.0713;
        SegmentRadiiOfGyration[SegmentLowerArm][2] = 0.0325;

        SegmentRadiiOfGyration[SegmentHand][0] = 0.0541;
        SegmentRadiiOfGyration[SegmentHand][1] = 0.0442;
        SegmentRadiiOfGyration[SegmentHand][2] = 0.0346;

        SegmentRadiiOfGyration[SegmentHead][0] = 0.0736;
        SegmentRadiiOfGyration[SegmentHead][1] = 0.0634;
        SegmentRadiiOfGyration[SegmentHead][2] = 0.0765;
    };

    RobotDynamics::Body create_body(SegmentName segment)
    {
        using namespace RobotDynamics;
        using namespace RobotDynamics::Math;

        Matrix3d inertia_C(Matrix3d::Zero());
        inertia_C(0, 0) = pow(SegmentRadiiOfGyration[segment][0] * SegmentLengths[segment], 2) * SegmentMass[segment];
        inertia_C(1, 1) = pow(SegmentRadiiOfGyration[segment][1] * SegmentLengths[segment], 2) * SegmentMass[segment];
        inertia_C(2, 2) = pow(SegmentRadiiOfGyration[segment][2] * SegmentLengths[segment], 2) * SegmentMass[segment];

        return RobotDynamics::Body(SegmentMass[segment], RobotDynamics::Math::Vector3d(SegmentCOM[segment][0], SegmentCOM[segment][1], SegmentCOM[segment][2]), inertia_C,
                                   RobotDynamics::Math::Vector3d(SegmentCOB[segment][0], SegmentCOB[segment][1], SegmentCOB[segment][2]), SegmentVolume[segment],
                                   RobotDynamics::Math::SpatialMatrixZero);
    }

    void generate()
    {
        using namespace RobotDynamics;
        using namespace RobotDynamics::Math;

        Body pelvis_body = create_body(SegmentPelvis);
        Body thigh_body = create_body(SegmentThigh);
        Body shank_body = create_body(SegmentShank);
        Body foot_body = create_body(SegmentFoot);
        Body middle_trunk_body = create_body(SegmentMiddleTrunk);
        Body upper_trunk_body = create_body(SegmentUpperTrunk);
        Body upperarm_body = create_body(SegmentUpperArm);
        Body lowerarm_body = create_body(SegmentLowerArm);
        Body hand_body = create_body(SegmentHand);
        Body head_body = create_body(SegmentHead);

        Matrix3d zero_matrix(Matrix3d::Zero(3, 3));
        Body null_body(0., Vector3d(0., 0., 0.), zero_matrix);
        null_body.mIsVirtual = true;

        Joint free_flyer(SpatialVector(0., 0., 0., 1., 0., 0.), SpatialVector(0., 0., 0., 0., 1., 0.), SpatialVector(0., 0., 0., 0., 0., 1.),
                         SpatialVector(0., 1., 0., 0., 0., 0.), SpatialVector(1., 0., 0., 0., 0., 0.), SpatialVector(0., 0., 1., 0., 0., 0.));

        Joint rot_yxz_emulated(SpatialVector(0., 1., 0., 0., 0., 0.), SpatialVector(1., 0., 0., 0., 0., 0.), SpatialVector(0., 0., 1., 0., 0., 0.));

        Joint trans_xyz = Joint(JointTypeTranslationXYZ);

        Joint rot_yxz_3dof = Joint(JointTypeEulerYXZ);

        Joint rot_yz(SpatialVector(0., 1., 0., 0., 0., 0.), SpatialVector(0., 0., 1., 0., 0., 0.));

        Joint rot_y(SpatialVector(0., 1., 0., 0., 0., 0.));

        Joint fixed(JointTypeFixed);

        // Generate emulated model
        model_emulated->gravity = MotionVector(0., 0., 0., 0., 0., -9.81);

        body_id_emulated[BodyPelvis] = model_emulated->addBody(0, Xtrans(Vector3d(0., 0., 0.)), free_flyer, pelvis_body, "pelvis");

        // right leg
        body_id_emulated[BodyThighRight] =
            model_emulated->addBody(body_id_emulated[BodyPelvis], Xtrans(Vector3d(0., -0.0872, 0.)), rot_yxz_emulated, thigh_body, "thigh_r");
        body_id_emulated[BodyShankRight] = model_emulated->appendBody(Xtrans(Vector3d(0., 0., -SegmentLengths[SegmentThigh])), rot_y, shank_body, "shank_r");
        body_id_emulated[BodyFootRight] = model_emulated->appendBody(Xtrans(Vector3d(0., 0., -SegmentLengths[SegmentShank])), rot_yz, foot_body, "foot_r");

        // left leg
        body_id_emulated[BodyThighLeft] =
            model_emulated->addBody(body_id_emulated[BodyPelvis], Xtrans(Vector3d(0., 0.0872, 0.)), rot_yxz_emulated, thigh_body, "thigh_l");
        body_id_emulated[BodyShankLeft] = model_emulated->appendBody(Xtrans(Vector3d(0., 0., -SegmentLengths[SegmentThigh])), rot_y, shank_body, "shank_l");
        body_id_emulated[BodyFootLeft] = model_emulated->appendBody(Xtrans(Vector3d(0., 0., -SegmentLengths[SegmentShank])), rot_yz, foot_body, "foot_l");

        // trunk
        body_id_emulated[BodyMiddleTrunk] = model_emulated->addBody(body_id_emulated[BodyPelvis], Xtrans(Vector3d(0., 0., SegmentLengths[SegmentPelvis])),
                                                                    rot_yxz_emulated, middle_trunk_body, "middletrunk");
        body_id_emulated[BodyUpperTrunk] =
            model_emulated->appendBody(Xtrans(Vector3d(0., 0., SegmentLengths[SegmentMiddleTrunk])), fixed, upper_trunk_body, "uppertrunk");

        // right arm
        body_id_emulated[BodyUpperArmRight] = model_emulated->addBody(body_id_emulated[BodyUpperTrunk], Xtrans(Vector3d(0., -0.1900, SegmentLengths[SegmentUpperTrunk])),
                                                                      rot_yxz_emulated, upperarm_body, "upperarm_r");
        body_id_emulated[BodyLowerArmRight] = model_emulated->appendBody(Xtrans(Vector3d(0., 0., -SegmentLengths[SegmentUpperArm])), rot_y, lowerarm_body, "lowerarm_r");
        body_id_emulated[BodyHandRight] = model_emulated->appendBody(Xtrans(Vector3d(0., 0., -SegmentLengths[SegmentLowerArm])), rot_yz, hand_body, "hand_r");

        // left arm
        body_id_emulated[BodyUpperArmLeft] = model_emulated->addBody(body_id_emulated[BodyUpperTrunk], Xtrans(Vector3d(0., 0.1900, SegmentLengths[SegmentUpperTrunk])),
                                                                     rot_yxz_emulated, upperarm_body, "upperarm_l");
        body_id_emulated[BodyLowerArmLeft] = model_emulated->appendBody(Xtrans(Vector3d(0., 0., -SegmentLengths[SegmentUpperArm])), rot_y, lowerarm_body, "lowerarm_l");
        body_id_emulated[BodyHandLeft] = model_emulated->appendBody(Xtrans(Vector3d(0., 0., -SegmentLengths[SegmentLowerArm])), rot_yz, hand_body, "hand_l");

        // head
        body_id_emulated[BodyHead] = model_emulated->addBody(body_id_emulated[BodyUpperTrunk], Xtrans(Vector3d(0., 0.1900, SegmentLengths[SegmentUpperTrunk])),
                                                             rot_yxz_emulated, upperarm_body, "head");

        // Generate 3dof model
        model_3dof->gravity = MotionVector(0., 0., 0., 0., 0., -9.81);

        unsigned int pelvis_trans = model_3dof->addBody(0, Xtrans(Vector3d(0., 0., 0.)), trans_xyz, null_body, "pelvis_trans_xyz");

        body_id_3dof[BodyPelvis] = model_3dof->addBody(pelvis_trans, Xtrans(Vector3d(0., 0., 0.)), rot_yxz_3dof, pelvis_body, "pelvis");
        //		body_id_3dof[BodyPelvis] = model_3dof->addBody (0, Xtrans (Vector3d (0., 0., 0.)), free_flyer, pelvis_body, "pelvis");

        // right leg
        body_id_3dof[BodyThighRight] = model_3dof->addBody(body_id_3dof[BodyPelvis], Xtrans(Vector3d(0., -0.0872, 0.)), rot_yxz_3dof, thigh_body, "thigh_r");
        body_id_3dof[BodyShankRight] = model_3dof->appendBody(Xtrans(Vector3d(0., 0., -SegmentLengths[SegmentThigh])), rot_y, shank_body, "shank_r");
        body_id_3dof[BodyFootRight] = model_3dof->appendBody(Xtrans(Vector3d(0., 0., -SegmentLengths[SegmentShank])), rot_yz, foot_body, "foot_r");

        // left leg
        body_id_3dof[BodyThighLeft] = model_3dof->addBody(body_id_3dof[BodyPelvis], Xtrans(Vector3d(0., 0.0872, 0.)), rot_yxz_3dof, thigh_body, "thigh_l");
        body_id_3dof[BodyShankLeft] = model_3dof->appendBody(Xtrans(Vector3d(0., 0., -SegmentLengths[SegmentThigh])), rot_y, shank_body, "shank_l");
        body_id_3dof[BodyFootLeft] = model_3dof->appendBody(Xtrans(Vector3d(0., 0., -SegmentLengths[SegmentShank])), rot_yz, foot_body, "foot_l");

        // trunk
        body_id_3dof[BodyMiddleTrunk] =
            model_3dof->addBody(body_id_3dof[BodyPelvis], Xtrans(Vector3d(0., 0., SegmentLengths[SegmentPelvis])), rot_yxz_3dof, middle_trunk_body, "middletrunk");
        body_id_3dof[BodyUpperTrunk] = model_3dof->appendBody(Xtrans(Vector3d(0., 0., SegmentLengths[SegmentMiddleTrunk])), fixed, upper_trunk_body, "uppertrunk");

        // right arm
        body_id_3dof[BodyUpperArmRight] = model_3dof->addBody(body_id_3dof[BodyUpperTrunk], Xtrans(Vector3d(0., -0.1900, SegmentLengths[SegmentUpperTrunk])),
                                                              rot_yxz_3dof, upperarm_body, "upperarm_r");
        body_id_3dof[BodyLowerArmRight] = model_3dof->appendBody(Xtrans(Vector3d(0., 0., -SegmentLengths[SegmentUpperArm])), rot_y, lowerarm_body, "lowerarm_r");
        body_id_3dof[BodyHandRight] = model_3dof->appendBody(Xtrans(Vector3d(0., 0., -SegmentLengths[SegmentLowerArm])), rot_yz, hand_body, "hand_r");

        // left arm
        body_id_3dof[BodyUpperArmLeft] =
            model_3dof->addBody(body_id_3dof[BodyUpperTrunk], Xtrans(Vector3d(0., 0.1900, SegmentLengths[SegmentUpperTrunk])), rot_yxz_3dof, upperarm_body, "upperarm_l");
        body_id_3dof[BodyLowerArmLeft] = model_3dof->appendBody(Xtrans(Vector3d(0., 0., -SegmentLengths[SegmentUpperArm])), rot_y, lowerarm_body, "lowerarm_l");
        body_id_3dof[BodyHandLeft] = model_3dof->appendBody(Xtrans(Vector3d(0., 0., -SegmentLengths[SegmentLowerArm])), rot_yz, hand_body, "hand_l");

        // head
        body_id_3dof[BodyHead] =
            model_3dof->addBody(body_id_3dof[BodyUpperTrunk], Xtrans(Vector3d(0., 0.1900, SegmentLengths[SegmentUpperTrunk])), rot_yxz_3dof, upperarm_body, "head");
    }

    void initConstraintSets()
    {
        using namespace RobotDynamics;
        using namespace RobotDynamics::Math;

        unsigned int foot_r_emulated = model_emulated->GetBodyId("foot_r");
        unsigned int foot_l_emulated = model_emulated->GetBodyId("foot_l");
        unsigned int hand_r_emulated = model_emulated->GetBodyId("hand_r");
        unsigned int hand_l_emulated = model_emulated->GetBodyId("hand_l");

        constraints_1B1C_emulated.addConstraint(foot_r_emulated, Vector3d(0.1, 0., -0.05), Vector3d(1., 0., 0.));
        constraints_1B1C_emulated.bind(*model_emulated);

        constraints_1B4C_emulated.addConstraint(foot_r_emulated, Vector3d(0.1, 0., -0.05), Vector3d(1., 0., 0.));
        constraints_1B4C_emulated.addConstraint(foot_r_emulated, Vector3d(0.1, 0., -0.05), Vector3d(0., 1., 0.));
        constraints_1B4C_emulated.addConstraint(foot_r_emulated, Vector3d(0.1, 0., -0.05), Vector3d(0., 0., 1.));
        constraints_1B4C_emulated.addConstraint(foot_r_emulated, Vector3d(-0.1, 0., -0.05), Vector3d(1., 0., 0.));
        constraints_1B4C_emulated.bind(*model_emulated);

        constraints_4B4C_emulated.addConstraint(foot_r_emulated, Vector3d(0.1, 0., -0.05), Vector3d(1., 0., 0.));
        constraints_4B4C_emulated.addConstraint(foot_r_emulated, Vector3d(0.1, 0., -0.05), Vector3d(0., 1., 0.));
        constraints_4B4C_emulated.addConstraint(foot_r_emulated, Vector3d(0.1, 0., -0.05), Vector3d(0., 0., 1.));
        constraints_4B4C_emulated.addConstraint(foot_r_emulated, Vector3d(-0.1, 0., -0.05), Vector3d(1., 0., 0.));

        constraints_4B4C_emulated.addConstraint(foot_l_emulated, Vector3d(0.1, 0., -0.05), Vector3d(1., 0., 0.));
        constraints_4B4C_emulated.addConstraint(foot_l_emulated, Vector3d(0.1, 0., -0.05), Vector3d(0., 1., 0.));
        constraints_4B4C_emulated.addConstraint(foot_l_emulated, Vector3d(0.1, 0., -0.05), Vector3d(0., 0., 1.));
        constraints_4B4C_emulated.addConstraint(foot_l_emulated, Vector3d(-0.1, 0., -0.05), Vector3d(1., 0., 0.));

        constraints_4B4C_emulated.addConstraint(hand_r_emulated, Vector3d(0.1, 0., -0.05), Vector3d(1., 0., 0.));
        constraints_4B4C_emulated.addConstraint(hand_r_emulated, Vector3d(0.1, 0., -0.05), Vector3d(0., 1., 0.));
        constraints_4B4C_emulated.addConstraint(hand_r_emulated, Vector3d(0.1, 0., -0.05), Vector3d(0., 0., 1.));
        constraints_4B4C_emulated.addConstraint(hand_r_emulated, Vector3d(-0.1, 0., -0.05), Vector3d(1., 0., 0.));

        constraints_4B4C_emulated.addConstraint(hand_l_emulated, Vector3d(0.1, 0., -0.05), Vector3d(1., 0., 0.));
        constraints_4B4C_emulated.addConstraint(hand_l_emulated, Vector3d(0.1, 0., -0.05), Vector3d(0., 1., 0.));
        constraints_4B4C_emulated.addConstraint(hand_l_emulated, Vector3d(0.1, 0., -0.05), Vector3d(0., 0., 1.));
        constraints_4B4C_emulated.addConstraint(hand_l_emulated, Vector3d(-0.1, 0., -0.05), Vector3d(1., 0., 0.));
        constraints_4B4C_emulated.bind(*model);

        unsigned int foot_r_3dof = model_3dof->GetBodyId("foot_r");
        unsigned int foot_l_3dof = model_3dof->GetBodyId("foot_l");
        unsigned int hand_r_3dof = model_3dof->GetBodyId("hand_r");
        unsigned int hand_l_3dof = model_3dof->GetBodyId("hand_l");

        constraints_1B1C_3dof.addConstraint(foot_r_3dof, Vector3d(0.1, 0., -0.05), Vector3d(1., 0., 0.));
        constraints_1B1C_3dof.bind(*model_3dof);

        constraints_1B4C_3dof.addConstraint(foot_r_3dof, Vector3d(0.1, 0., -0.05), Vector3d(1., 0., 0.));
        constraints_1B4C_3dof.addConstraint(foot_r_3dof, Vector3d(0.1, 0., -0.05), Vector3d(0., 1., 0.));
        constraints_1B4C_3dof.addConstraint(foot_r_3dof, Vector3d(0.1, 0., -0.05), Vector3d(0., 0., 1.));
        constraints_1B4C_3dof.addConstraint(foot_r_3dof, Vector3d(-0.1, 0., -0.05), Vector3d(1., 0., 0.));
        constraints_1B4C_3dof.bind(*model_3dof);

        constraints_4B4C_3dof.addConstraint(foot_r_3dof, Vector3d(0.1, 0., -0.05), Vector3d(1., 0., 0.));
        constraints_4B4C_3dof.addConstraint(foot_r_3dof, Vector3d(0.1, 0., -0.05), Vector3d(0., 1., 0.));
        constraints_4B4C_3dof.addConstraint(foot_r_3dof, Vector3d(0.1, 0., -0.05), Vector3d(0., 0., 1.));
        constraints_4B4C_3dof.addConstraint(foot_r_3dof, Vector3d(-0.1, 0., -0.05), Vector3d(1., 0., 0.));

        constraints_4B4C_3dof.addConstraint(foot_l_3dof, Vector3d(0.1, 0., -0.05), Vector3d(1., 0., 0.));
        constraints_4B4C_3dof.addConstraint(foot_l_3dof, Vector3d(0.1, 0., -0.05), Vector3d(0., 1., 0.));
        constraints_4B4C_3dof.addConstraint(foot_l_3dof, Vector3d(0.1, 0., -0.05), Vector3d(0., 0., 1.));
        constraints_4B4C_3dof.addConstraint(foot_l_3dof, Vector3d(-0.1, 0., -0.05), Vector3d(1., 0., 0.));

        constraints_4B4C_3dof.addConstraint(hand_r_3dof, Vector3d(0.1, 0., -0.05), Vector3d(1., 0., 0.));
        constraints_4B4C_3dof.addConstraint(hand_r_3dof, Vector3d(0.1, 0., -0.05), Vector3d(0., 1., 0.));
        constraints_4B4C_3dof.addConstraint(hand_r_3dof, Vector3d(0.1, 0., -0.05), Vector3d(0., 0., 1.));
        constraints_4B4C_3dof.addConstraint(hand_r_3dof, Vector3d(-0.1, 0., -0.05), Vector3d(1., 0., 0.));

        constraints_4B4C_3dof.addConstraint(hand_l_3dof, Vector3d(0.1, 0., -0.05), Vector3d(1., 0., 0.));
        constraints_4B4C_3dof.addConstraint(hand_l_3dof, Vector3d(0.1, 0., -0.05), Vector3d(0., 1., 0.));
        constraints_4B4C_3dof.addConstraint(hand_l_3dof, Vector3d(0.1, 0., -0.05), Vector3d(0., 0., 1.));
        constraints_4B4C_3dof.addConstraint(hand_l_3dof, Vector3d(-0.1, 0., -0.05), Vector3d(1., 0., 0.));
        constraints_4B4C_3dof.bind(*model_3dof);
    }

    void randomizeStates()
    {
        for (int i = 0; i < q.size(); i++)
        {
            q[i] = 0.4 * M_PI * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
            qdot[i] = 0.5 * M_PI * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
            tau[i] = 0.5 * M_PI * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
            qddot[i] = 0.5 * M_PI * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        }

        qddot_emulated = qddot;
        qddot_3dof = qddot;
    }

    Human36()
    {
        srand(time(NULL));
    }

    void SetUp()
    {
        using namespace RobotDynamics;
        using namespace RobotDynamics::Math;

        initParameters();
        model_emulated.reset(new RobotDynamics::Model());
        model_3dof.reset(new RobotDynamics::Model());
        model_emulated->fluidDensity = fluid_density;
        model_3dof->fluidDensity = fluid_density;
        model = model_emulated;
        generate();
        initConstraintSets();

        q = VectorNd::Zero(model_emulated->q_size);
        qdot = VectorNd::Zero(model_emulated->qdot_size);
        qddot = VectorNd::Zero(model_emulated->qdot_size);
        tau = VectorNd::Zero(model_emulated->qdot_size);

        qddot_emulated = VectorNd::Zero(model_emulated->qdot_size);
        qddot_3dof = VectorNd::Zero(model_emulated->qdot_size);
    };

    void TearDown()
    {
        EXPECT_TRUE(unit_test_utils::checkModelZeroVectorsAndMatrices(*model_emulated));
        EXPECT_TRUE(unit_test_utils::checkModelZeroVectorsAndMatrices(*model_3dof));
    }
};

#endif
