//
// Created by LiuQiang on 2025/1/5.
//

#ifndef SCENE_JOINTTYPE_H
#define SCENE_JOINTTYPE_H

enum class JointType
{
    UNKNOWN = 0,
    REVOLUTE,
    CONTINUOUS,
    PRISMATIC,
    PLANAR,
    FLOAT,
    FIXED
};

#endif // SCENE_JOINTTYPE_H
