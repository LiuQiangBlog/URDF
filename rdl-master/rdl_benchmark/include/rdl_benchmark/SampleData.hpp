// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// Copyright (c) 2011-2016 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#ifndef _RDL_BENCHMARK_SAMPLE_DATA_H__
#define _RDL_BENCHMARK_SAMPLE_DATA_H__

#include <rdl_dynamics/rdl_eigenmath.hpp>

struct SampleData
{
    SampleData() : count(0), q(NULL), qdot(NULL), qddot(NULL), tau(NULL)
    {
    }

    ~SampleData()
    {
        deleteData();
    }

    SampleData(const SampleData& data)
    {
        count = data.count;

        q = new RobotDynamics::Math::VectorNd[count];
        qdot = new RobotDynamics::Math::VectorNd[count];
        qddot = new RobotDynamics::Math::VectorNd[count];
        tau = new RobotDynamics::Math::VectorNd[count];

        for (unsigned int si = 0; si < count; si++)
        {
            q[si] = data.q[si];
            qdot[si] = data.qdot[si];
            qddot[si] = data.qddot[si];
            tau[si] = data.tau[si];
        }
    }

    SampleData& operator=(const SampleData& data)
    {
        if (this != &data)
        {
            deleteData();
            *this = SampleData(data);
        }
        return *this;
    }

    unsigned int count;
    RobotDynamics::Math::VectorNd* q;
    RobotDynamics::Math::VectorNd* qdot;
    RobotDynamics::Math::VectorNd* qddot;
    RobotDynamics::Math::VectorNd* tau;

    void deleteData()
    {
        count = 0;

        if (q)
        {
            delete[] q;
        }
        q = NULL;

        if (qdot)
        {
            delete[] qdot;
        }
        qdot = NULL;

        if (qddot)
        {
            delete[] qddot;
        }
        qddot = NULL;

        if (tau)
        {
            delete[] tau;
        }
        tau = NULL;
    }

    void fillRandom(unsigned int dof_count, unsigned int sample_count)
    {
        deleteData();
        count = sample_count;

        q = new RobotDynamics::Math::VectorNd[count];
        qdot = new RobotDynamics::Math::VectorNd[count];
        qddot = new RobotDynamics::Math::VectorNd[count];
        tau = new RobotDynamics::Math::VectorNd[count];

        for (unsigned int si = 0; si < count; si++)
        {
            q[si].resize(dof_count);
            qdot[si].resize(dof_count);
            qddot[si].resize(dof_count);
            tau[si].resize(dof_count);

            for (unsigned int i = 0; i < dof_count; i++)
            {
                q[si][i] = (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) * 2. - 1.;
                qdot[si][i] = (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) * 2. - 1.;
                qddot[si][i] = (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) * 2. - 1.;
                tau[si][i] = (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) * 2. - 1.;
            }
        }
    }
};

#endif
