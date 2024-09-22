// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// Copyright (c) 2011-2016 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#ifndef RDL_BODY_HPP
#define RDL_BODY_HPP

#include <rdl_dynamics/RigidBodyInertia.hpp>
#include <rdl_dynamics/rdl_mathutils.hpp>
#include <assert.h>
#include <iostream>
#include <rdl_dynamics/RdlExceptions.hpp>

namespace RobotDynamics
{
/**
 * @struct A compact struct for storing the drag data for a body
 */
struct DragData
{
    DragData(const std::vector<double>& linear_drag, const std::vector<double>& quadratic_drag)
    {
        assert(linear_drag.size() == 6 && quadratic_drag.size() == 6);
        for (unsigned int i = 0; i < 6; i++)
        {
            this->linearDrag[i] = linear_drag[i];
            this->quadraticDrag[i] = quadratic_drag[i];
        }
    }

    DragData(const Math::SpatialVector& linear_drag, const Math::SpatialVector& quadratic_drag)
    {
        this->linearDrag = linear_drag;
        this->quadraticDrag = quadratic_drag;
    }

    DragData()
    {
        zero();
    }

    DragData(const DragData& dragData)
    {
        this->linearDrag = dragData.linearDrag;
        this->quadraticDrag = dragData.quadraticDrag;
    }

    void zero()
    {
        linearDrag.setZero();
        quadraticDrag.setZero();
    }

    DragData& operator=(const DragData& dragData)
    {
        this->linearDrag = dragData.linearDrag;
        this->quadraticDrag = dragData.quadraticDrag;
        return *this;
    }

    void operator+=(const DragData& dragData)
    {
        linearDrag += dragData.linearDrag;
        quadraticDrag += dragData.quadraticDrag;
    }

    Math::SpatialVector linearDrag;    /**< 6 coefficients for the linear portion of the drag effect */
    Math::SpatialVector quadraticDrag; /**< 6 coefficients for the quadratic portion of the drag effect */
};

inline DragData operator+(DragData dragData1, const DragData& dragData2)
{
    dragData1 += dragData2;
    return dragData1;
}

/** \brief Describes all properties of a single body
 *
 * A Body contains information about mass, the location of its center of
 * mass, the ineria tensor in the center of mass as well as information
 * for hydrodynamics such as center of buoyancy, volume, and shape parameters
 * for modeling fluid added mass/drag effects. This class is
 * designed to use the given information and transform it such that it can
 * directly be used by the spatial algebra.
 */
struct Body
{
    Body()
      : mMass(0.)
      , mCenterOfMass(0., 0., 0.)
      , mInertia(Math::Matrix3d::Zero(3, 3))
      , volume(0.)
      , mCenterOfBuoyancy(0., 0., 0.)
      , addedMassMatrix(Math::SpatialMatrixZero)
      , mIsVirtual(false)
    {
        dragData.zero();
    };

    Body(const Body& body)
      : mMass(body.mMass)
      , mCenterOfMass(body.mCenterOfMass)
      , mInertia(body.mInertia)
      , volume(body.volume)
      , mCenterOfBuoyancy(body.mCenterOfBuoyancy)
      , addedMassMatrix(body.addedMassMatrix)
      , dragData(body.dragData)
      , mIsVirtual(body.mIsVirtual){};

    Body& operator=(const Body& body)
    {
        if (this != &body)
        {
            mMass = body.mMass;
            mInertia = body.mInertia;
            mCenterOfMass = body.mCenterOfMass;
            mIsVirtual = body.mIsVirtual;
            mCenterOfBuoyancy = body.mCenterOfBuoyancy;
            volume = body.volume;
            addedMassMatrix = body.addedMassMatrix;
            dragData = body.dragData;
        }

        return *this;
    }

    /** \brief Constructs a body from mass, center of mass, radii of gyration, the center of buoyancy,
     * added mass matrix, drag data, and the volume taken up by the body
     *
     * This constructor eases the construction of a new body as all the
     * required parameters can be specified as parameters to the
     * constructor. These are then used to generate the spatial inertia
     * matrix which is expressed at the origin.
     *
     * \param mass the mass of the body
     * \param com  the position of the center of mass in the bodies coordinates
     * \param gyration_radii the radii of gyration at the center of mass of the body
     * \param centerOfBuoyancy The center of buoyancy for the body
     * \param volume The volume taken up by the body
     * \param addedMassMatrix The hydrodynamic added mass matrix
     * \param dragData Hydrodynamic drag data struct for the body
     */
    Body(const double& mass, const Math::Vector3d& com, const Math::Vector3d& gyration_radii, const Math::Vector3d& centerOfBuoyancy, const double volume,
         const Math::SpatialMatrix& addedMassMatrix, const DragData& dragData)
      : mMass(mass), mCenterOfMass(com), volume(volume), mCenterOfBuoyancy(centerOfBuoyancy), addedMassMatrix(addedMassMatrix), dragData(dragData), mIsVirtual(false)
    {
        mInertia = Math::Matrix3d(gyration_radii[0], 0., 0., 0., gyration_radii[1], 0., 0., 0., gyration_radii[2]);
    }

    /** \brief Constructs a body from mass, center of mass, radii of gyration, the center of buoyancy,
     * and hydrodynamic parameters
     *
     * This constructor eases the construction of a new body as all the
     * required parameters can be specified as parameters to the
     * constructor. These are then used to generate the spatial inertia
     * matrix which is expressed at the origin.
     *
     * \param mass the mass of the body
     * \param com  the position of the center of mass in the bodies coordinates
     * \param gyration_radii the radii of gyration at the center of mass of the body
     * \param centerOfBuoyancy The center of buoyancy for the body
     * \param volume The volume taken up by the body
     * \param addedMassMatrix The hydrodynamic added mass matrix for the body
     */
    Body(const double& mass, const Math::Vector3d& com, const Math::Vector3d& gyration_radii, const Math::Vector3d& centerOfBuoyancy, const double volume,
         const Math::SpatialMatrix& addedMassMatrix)
      : mMass(mass), mCenterOfMass(com), volume(volume), mCenterOfBuoyancy(centerOfBuoyancy), addedMassMatrix(addedMassMatrix), mIsVirtual(false)
    {
        dragData.zero();
        mInertia = Math::Matrix3d(gyration_radii[0], 0., 0., 0., gyration_radii[1], 0., 0., 0., gyration_radii[2]);
    }

    /** \brief Constructs a body from mass, center of mass, radii of gyration. Hydrodynamic terms are set such that
     * they will have no effect on the dynamics
     *
     * This constructor eases the construction of a new body as all the
     * required parameters can be specified as parameters to the
     * constructor. These are then used to generate the spatial inertia
     * matrix which is expressed at the origin.
     *
     * \param mass the mass of the body
     * \param com  the position of the center of mass in the bodies coordinates
     * \param gyration_radii the radii of gyration at the center of mass of the body
     */
    Body(const double& mass, const Math::Vector3d& com, const Math::Vector3d& gyration_radii)
      : mMass(mass), mCenterOfMass(com), volume(0.), mCenterOfBuoyancy(Math::Vector3dZero), mIsVirtual(false)
    {
        mInertia = Math::Matrix3d(gyration_radii[0], 0., 0., 0., gyration_radii[1], 0., 0., 0., gyration_radii[2]);
        dragData.zero();
        addedMassMatrix.setZero();
    }

    /** \brief Constructs a body from mass, center of mass, a 3x3 inertia matrix, the center of buoyancy,
     * and hydrodynamic parameters
     *
     * This constructor eases the construction of a new body as all the
     * required parameters can simply be specified as parameters to the
     * constructor. These are then used to generate the spatial inertia
     * matrix which is expressed at the origin.
     *
     * \param mass the mass of the body
     * \param com  the position of the center of mass in the bodies coordinates
     * \param inertia_C the inertia at the center of mass
     * \param centerOfBuoyancy The center of buoyancy for the body
     * \param volume The volume taken up by the body
     * \param addedMassMatrix Hydrodynamic added mass matrix for the body
     * \param dragData The drag data for this body
     */
    Body(const double& mass, const Math::Vector3d& com, const Math::Matrix3d& inertia_C, const Math::Vector3d& centerOfBuoyancy, const double volume,
         const Math::SpatialMatrix& addedMassMatrix, const DragData& dragData)
      : mMass(mass)
      , mCenterOfMass(com)
      , mInertia(inertia_C)
      , volume(volume)
      , mCenterOfBuoyancy(centerOfBuoyancy)
      , addedMassMatrix(addedMassMatrix)
      , dragData(dragData)
      , mIsVirtual(false)
    {
    }

    /** \brief Constructs a body from mass, center of mass, a 3x3 inertia matrix, the center of buoyancy,
     * and hydrodynamic parameters
     *
     * This constructor eases the construction of a new body as all the
     * required parameters can simply be specified as parameters to the
     * constructor. These are then used to generate the spatial inertia
     * matrix which is expressed at the origin.
     *
     * \param mass the mass of the body
     * \param com  the position of the center of mass in the bodies coordinates
     * \param inertia_C the inertia at the center of mass
     * \param centerOfBuoyancy The center of buoyancy for the body
     * \param volume The volume taken up by the body
     * \param addedMassMatrix The hydrodynamic added mass matrix for the body
     */
    Body(const double& mass, const Math::Vector3d& com, const Math::Matrix3d& inertia_C, const Math::Vector3d& centerOfBuoyancy, const double volume,
         const Math::SpatialMatrix addedMassMatrix)
      : mMass(mass), mCenterOfMass(com), mInertia(inertia_C), volume(volume), mCenterOfBuoyancy(centerOfBuoyancy), addedMassMatrix(addedMassMatrix), mIsVirtual(false)
    {
        dragData.zero();
    }

    /** \brief Constructs a body from mass, center of mass, a 3x3 inertia matrix. Hydrodynamic terms are set such that
     * they will have no dynamic effects
     *
     * This constructor eases the construction of a new body as all the
     * required parameters can simply be specified as parameters to the
     * constructor. These are then used to generate the spatial inertia
     * matrix which is expressed at the origin.
     *
     * \param mass the mass of the body
     * \param com  the position of the center of mass in the bodies coordinates
     * \param inertia_C the inertia at the center of mass
     */
    Body(const double& mass, const Math::Vector3d& com, const Math::Matrix3d& inertia_C)
      : mMass(mass), mCenterOfMass(com), mInertia(inertia_C), volume(0.), mCenterOfBuoyancy(Math::Vector3dZero), mIsVirtual(false)
    {
        addedMassMatrix.setZero();
        dragData.zero();
    }

    /** \brief Joins inertial parameters of two bodies to create a composite
     * body.
     *
     * This function can be used to join inertial parameters of two bodies
     * to create a composite body that has the inertial properties as if the
     * two bodies were joined by a fixed joint.
     *
     * \note The drag terms and added mass matrices for both bodies will simply be added
     * together
     *
     * \param transform The frame transformation from the origin of the
     * original body to the origin of the added body
     * \param other_body The other body that will be merged with *this.
     */
    void join(const Math::SpatialTransform& transform, const Body& other_body)
    {
        // nothing to do if we join a massles body to the current.
        if (other_body.mMass == 0. && other_body.mInertia == Math::Matrix3d::Zero())
        {
            return;
        }

        double other_mass = other_body.mMass;
        double new_mass = mMass + other_mass;

        double other_volume = other_body.volume;
        double new_volume = volume + other_volume;

        if (new_mass == 0.)
        {
            throw RdlException("Error: cannot join bodies as both have zero mass!");
        }

        Math::Vector3d other_com = transform.E.transpose() * other_body.mCenterOfMass + transform.r;
        Math::Vector3d new_com = (1. / new_mass) * (mMass * mCenterOfMass + other_mass * other_com);
        Math::Vector3d new_cob = mCenterOfBuoyancy;
        if (new_volume != 0.)
        {
            Math::Vector3d other_cob = transform.E.transpose() * other_body.mCenterOfBuoyancy + transform.r;
            new_cob = (1. / new_volume) * (volume * mCenterOfBuoyancy + other_volume * other_cob);
        }

        // We have to transform the inertia of other_body to the new COM. This
        // is done in 4 steps:
        //
        // 1. Transform the inertia from other origin to other COM
        // 2. Rotate the inertia that it is aligned to the frame of this body
        // 3. Transform inertia of other_body to the origin of the frame of
        // this body
        // 4. Sum the two inertias
        // 5. Transform the summed inertia to the new COM

        Math::RigidBodyInertia other_rbi = Math::createFromMassComInertiaC(other_body.mMass, other_body.mCenterOfMass, other_body.mInertia);
        Math::RigidBodyInertia this_rbi = Math::createFromMassComInertiaC(mMass, mCenterOfMass, mInertia);

        Math::Matrix3d inertia_other = other_rbi.toMatrix().block<3, 3>(0, 0);

        // 1. Transform the inertia from other origin to other COM
        Math::Matrix3d other_com_cross = other_body.mCenterOfMass.toTildeForm();
        Math::Matrix3d inertia_other_com = inertia_other - other_mass * other_com_cross * other_com_cross.transpose();

        // 2. Rotate the inertia that it is aligned to the frame of this body
        Math::Matrix3d inertia_other_com_rotated = transform.E.transpose() * inertia_other_com * transform.E;

        // 3. Transform inertia of other_body to the origin of the frame of this body
        Math::Matrix3d inertia_other_com_rotated_this_origin = Math::parallel_axis(inertia_other_com_rotated, other_mass, other_com);

        // 4. Sum the two inertias
        Math::Matrix3d inertia_summed = Math::Matrix3d(this_rbi.toMatrix().block<3, 3>(0, 0)) + inertia_other_com_rotated_this_origin;

        // 5. Transform the summed inertia to the new COM
        Math::Matrix3d new_inertia = inertia_summed - new_mass * new_com.toTildeForm() * new_com.toTildeForm().transpose();

        Math::SpatialMatrix new_added_mass_matrix = this->addedMassMatrix + other_body.addedMassMatrix;
        DragData new_drag_data = this->dragData + other_body.dragData;

        *this = Body(new_mass, new_com, new_inertia, new_cob, new_volume, new_added_mass_matrix, new_drag_data);
    }

    ~Body(){};

    /// \brief The mass of the body
    double mMass;
    /// \brief The position of the center of mass in body coordinates
    Math::Vector3d mCenterOfMass;
    /// \brief Inertia matrix at the center of mass
    Math::Matrix3d mInertia;

    double volume;                       /**< Volume taken up by body. */
    Math::Vector3d mCenterOfBuoyancy;    /**< Center of buoyancy for a body */
    Math::SpatialMatrix addedMassMatrix; /**< The hydrodynamic added mass for the body */
    DragData dragData;                   /**< The hydrodynamic drag data for the body */
    bool mIsVirtual;
};

/** \brief Keeps the information of a body and how it is attached to another body.
 *
 * When using fixed bodies, i.e. a body that is attached to anothe via a
 * fixed joint, the attached body is merged onto its parent. By doing so
 * adding fixed joints do not have an impact on runtime.
 */
struct FixedBody
{
    /// \brief The mass of the body
    double mMass;
    /// \brief The position of the center of mass in body coordinates
    Math::Vector3d mCenterOfMass;
    /// \brief The spatial inertia that contains both mass and inertia information
    Math::Matrix3d mInertia;
    double volume;                       /**< Volume taken up by body. */
    Math::Vector3d mCenterOfBuoyancy;    /**< Center of buoyancy for a body */
    Math::SpatialMatrix addedMassMatrix; /**< The hydrodynamic added mass for the body */
    DragData dragData;                   /**< The hydrodynamic drag data for the body */

    /// \brief Id of the movable body that this fixed body is attached to.
    unsigned int mMovableParent;
    /// \brief Transforms spatial quantities expressed for the parent to the
    // fixed body.
    Math::SpatialTransform mParentTransform;
    Math::SpatialTransform mBaseTransform;

    static FixedBody CreateFromBody(const Body& body)
    {
        FixedBody fbody;

        fbody.mMass = body.mMass;
        fbody.mCenterOfMass = body.mCenterOfMass;
        fbody.mInertia = body.mInertia;
        fbody.mCenterOfBuoyancy = body.mCenterOfBuoyancy;
        fbody.volume = body.volume;
        fbody.addedMassMatrix = body.addedMassMatrix;
        fbody.dragData = body.dragData;

        return fbody;
    }
};
}  // namespace RobotDynamics

#endif
