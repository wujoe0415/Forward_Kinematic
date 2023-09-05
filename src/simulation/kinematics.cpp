#include "simulation/kinematics.h"

#include "Eigen/Dense"
#include <iostream>
#include "acclaim/bone.h"
#include "util/helper.h"
#include <algorithm>
#include <stack>
#include <map>
#include <cmath>

#define M_PI 3.14159

namespace kinematics {
void forwardSolver(const acclaim::Posture& posture, acclaim::Bone* bone) {
    // TODO#1 (FK)
    // You should set these variables:
    //     bone->start_position = Eigen::Vector4d::Zero();
    //     bone->end_position = Eigen::Vector4d::Zero();
    //     bone->rotation = Eigen::Matrix4d::Zero();
    // The sample above just set everything to zero
    // Hint:
    //   1. posture.bone_translations, posture.bone_rotations
    // Note:
    //   1. This function will be called with bone == root bone of the skeleton
    //   2. we use 4D vector to represent 3D vector, so keep the last dimension as "0"
    //   3. util::rotate{Degree | Radian} {XYZ | ZYX}
    //      e.g. rotateDegreeXYZ(x, y, z) means:
    //      x, y and z are presented in degree rotate z degrees along z - axis first, then y degrees along y - axis, and then x degrees along x - axis 



    bone->start_position = Eigen::Vector4d::Zero();
    bone->end_position = Eigen::Vector4d::Zero();
    bone->rotation = Eigen::Matrix4d::Zero();



    std::map<int, bool> visit;
    std::stack<acclaim::Bone*> q;
    visit[bone->idx] = true;
    bone->start_position = posture.bone_translations[bone->idx];
    bone->end_position = posture.bone_translations[bone->idx];
    q.push(bone);

    while (!q.empty()) {
        acclaim::Bone* t = q.top();
        q.pop();
        if (t->idx != 0) {
            t->start_position = t->parent->end_position;
            Eigen::Quaterniond localRotation = util::rotateDegreeZYX(posture.bone_rotations[t->idx].x(), posture.bone_rotations[t->idx].y(), posture.bone_rotations[t->idx].z());
            Eigen::Affine3d rot = t->rot_parent_current * Eigen::Affine3d(localRotation);
            for (acclaim::Bone* itr = t->parent; itr != nullptr; itr = itr->parent) {
                Eigen::Quaterniond parentLocal = util::rotateDegreeZYX(posture.bone_rotations[itr->idx].x(), posture.bone_rotations[itr->idx].y(), posture.bone_rotations[itr->idx].z());
                rot = itr->rot_parent_current * parentLocal * rot;
            }

            Eigen::Vector4d dir = t->dir * t->length;
            t->end_position = rot.matrix() * dir + posture.bone_translations[t->idx] + t->start_position;
            
            t->rotation = rot;
        }
        for (acclaim::Bone* itr = t->child; itr != nullptr; itr = itr->sibling) {
            if (visit.find(itr->idx) == visit.end()) {
                visit[itr->idx] = true;
                q.push(itr);
            }
        }
    }
}

std::vector<acclaim::Posture> timeWarper(const std::vector<acclaim::Posture>& postures, int allframe_old, int allframe_new) {

    int total_frames = static_cast<int>(postures.size());
    int total_bones = static_cast<int>(postures[0].bone_rotations.size());
    std::vector<acclaim::Posture> new_postures;
    double scaler = (double)(allframe_old - 1)/(allframe_new - 1);
    for (int i = 0; i <= allframe_new; ++i) {
        acclaim::Posture new_poseture(total_bones);
        for (int j = 0; j < total_bones; ++j) {

            // TODO#2 (Time warping)
            // original: |--------------|
            // new     : |----------------------|
            // OR
            // original: |--------------|
            // new     : |-------|
            // You should set these variables:
            //     new_postures[i].bone_translations[j] = Eigen::Vector4d::Zero();
            //     new_postures[i].bone_rotations[j] = Eigen::Vector4d::Zero();
            // The sample above just set everything to zero
            // Hint:
            //   1. Scale the frames.
            //   2. You can use linear interpolation with translations.
            //   3. You should use spherical linear interpolation for rotations.


            new_poseture.bone_translations[j] = Eigen::Vector4d::Zero();
            new_poseture.bone_rotations[j] = Eigen::Vector4d::Zero();
            int leftFrame = i * scaler;
            int rightFrame = i * scaler + 1;
            if (leftFrame > allframe_old)
                leftFrame = allframe_old;
            if (rightFrame > allframe_old)
                rightFrame = allframe_old;

            //std::cout << allframe_old << " " << leftFrame << " " << rightFrame<<std::endl;

            double ratio = i * scaler - (int)(i * scaler);
            //std::cout << ratio << " \n";
            new_poseture.bone_translations[j] 
                = (1 - ratio) * postures[leftFrame].bone_translations[j] +
                       ratio * postures[rightFrame].bone_translations[j];
            Eigen::Quaterniond left = util::rotateDegreeXYZ(postures[leftFrame].bone_rotations[j].x(), postures[leftFrame].bone_rotations[j].y(), postures[leftFrame].bone_rotations[j].z());
            Eigen::Quaterniond right = util::rotateDegreeXYZ(postures[rightFrame].bone_rotations[j].x(), postures[rightFrame].bone_rotations[j].y(), postures[rightFrame].bone_rotations[j].z());
            
            Eigen::Vector3d target = left.slerp(ratio, right).toRotationMatrix().eulerAngles(0, 1, 2) * 180 / M_PI;
            new_poseture.bone_rotations[j] = Eigen::Vector4d(target.x(), target.y(), target.z(), 0);
        }

        new_postures.push_back(new_poseture);
    }
    return new_postures;
}
}  // namespace kinematics
