// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drive.Drive;

//Inspired by Team 2137's AutoAline code

/** Add your docs here. */
public class AutoAline {

  private static final double JoystickScalar = 2.500000;

  public enum Target {
    LEFT_POLE,
    RIGHT_POLE,
    ALGAE
  }

   private static final Map<Target, List<Pose2d>> targetToPoseData = Map.of(
        // Left reef poles
        Target.LEFT_POLE, FieldConstants.Reef.leftRobotBranchPoses,

        // Right reef poles
        Target.RIGHT_POLE, FieldConstants.Reef.rightRobotBranchPoses,

        // Locations for removing algae
        Target.ALGAE, FieldConstants.Reef.centerFaces
    );

  private static Pose2d target; // The currently targeted position (can be null)
  private static Pose2d lastTargeted = new Pose2d(); // The most recently targeted position (not null)

  public static Pose2d getActiveTarget() {
        return target;
  }

  public static Pose2d getLastTargeted() {
        return lastTargeted;
  }
  public static int mapToPoseId(Target targetType, Drive drive, Translation2d motionVector) {
        List<Pose2d> poseData = targetToPoseData.get(targetType);
        return getNearestPose(drive.getPose(), motionVector, poseData);
    }
    
    public static Pose2d fromPoseId(int id, Target targetType) {
      List<Pose2d> poseData = targetToPoseData.get(targetType);
      return poseData.get(id);
  }


}
