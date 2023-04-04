// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Util/AprilTagWrapper.h"

AprilTagWrapper::AprilTagWrapper() {
   
}

std::optional<photonlib::EstimatedRobotPose> AprilTagWrapper::getEstimatedGlobalPose() {  //frc::Pose3d prevEstimatedRobotPose
    return std::nullopt; 

    std::optional<photonlib::EstimatedRobotPose> result1 = estimator1.Update();
    std::optional<photonlib::EstimatedRobotPose> result2 = estimator2.Update();
    if (result1.has_value()) {
        if (!result2.has_value())  
            return result1;
        return result1.value().targetsUsed.size() > result2.value().targetsUsed.size() ? result1 : result2; // Combine result1 + result2.
    } else {
        if (!result2.has_value()) 
            return std::nullopt;
        return result2;
    }  
}
