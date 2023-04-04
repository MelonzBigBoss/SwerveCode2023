#include "Autos.h"

frc2::CommandPtr autos::Nothing() {
  return frc2::SequentialCommandGroup(std::move(frc2::WaitCommand(1_s))).ToPtr(); 
}
 
frc2::CommandPtr autos::PathPlannerAutoTest(Swerve* swerve) {
  std::vector<PathPlannerTrajectory> examplePath = PathPlanner::loadPathGroup("PathB", {PathConstraints {1_mps, 1_mps_sq}});
  std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap; 
  eventMap.emplace("marker1", std::make_shared<frc2::WaitCommand>(2_s));
  SwerveAutoBuilder autoBuilder(
      [swerve]() { return swerve->GetPose(); }, // pose function
      [swerve](auto initPose) {swerve->resetPose(initPose);}, // reset odometry function
      PIDConstants(1.0, 0.0, 0.0), // translation
      PIDConstants(0.5, 0.0, 0.0), // rotation
      [swerve](auto speeds) { swerve->DriveFieldRelative(speeds); }, //Drive function, takes in Chassis Speeds
      eventMap, // Our event map
      { swerve }, // Requirements
      false //Mirrored?
  );        
  return autoBuilder.fullAuto(examplePath);
}

frc2::CommandPtr autos::PathPlannerAutoTestB(Swerve* swerve) {
  
  std::vector<PathPlannerTrajectory> examplePath = PathPlanner::loadPathGroup("PathA", {PathConstraints {1_mps, 1_mps_sq}});
  std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap; 
  eventMap.emplace("marker1", std::make_shared<frc2::WaitCommand>(1_s));
  SwerveAutoBuilder autoBuilder(
      [swerve]() { return swerve->GetPose(); }, // pose function
      [swerve](auto initPose) {swerve->resetPose(initPose);}, // reset odometry function
      PIDConstants(1.0, 0.0, 0.0), // translation
      PIDConstants(0.5, 0.0, 0.0), // rotation
      [swerve](auto speeds) { swerve->DriveFieldRelative(speeds); }, //Drive function, takes in Chassis Speeds
      eventMap, // Our event map
      { swerve }, // Requirements
      false //Mirrored?
  );        
   return autoBuilder.fullAuto(examplePath);
}
