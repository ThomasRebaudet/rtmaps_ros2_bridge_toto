#pragma once

#include <Eigen/Core>

#include "maps.hpp"

#define MAX_OBJECTS 100

struct Detection {
    Eigen::Vector3f box_center_ = Eigen::Vector3f::Zero();
    Eigen::Vector3f box_size_ = Eigen::Vector3f::Zero();
    float box_angle_ = 0.0f;
    Eigen::Vector3f centroid_ = Eigen::Vector3f::Zero();
    Eigen::Vector3f velocity_ = Eigen::Vector3f::Zero();
    int num_pts_ = 0;
    int age_ = 0;
    int objectid_ = 0;
    uint16_t classid_ = 0;
  };

// The RTMaps input filter for the structure MyNewStructure
const MAPSTypeFilterBase MAPSFilterDetections = MAPS_FILTER_USER_STRUCTURE(Detection);