// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.StateController.Branch;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.AllianceFlipUtil;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

// Inspired by Team 2137's AutoAline code

/** Add your docs here. */
public class AutoAline {

  private static final double JoystickScalar = 2.500000;

  public enum Target {
    LEFT_POLE,
    RIGHT_POLE,
    ALGAE
  }

  private static final Map<Target, List<Pose2d>> targetToPoseData =
      Map.of(
          // Left reef poles
          Target.LEFT_POLE, FieldConstants.Reef.leftRobotBranchPoses,

          // Right reef poles
          Target.RIGHT_POLE, FieldConstants.Reef.rightRobotBranchPoses,

          // Locations for removing algae
          Target.ALGAE, FieldConstants.Reef.centerFaces);

  private static Pose2d target; // The currently targeted position (can be null)
  private static Pose2d lastTargeted =
      new Pose2d(); // The most recently targeted position (not null)

  private static PathPlannerPath targetPath;
  // private static PathPlannerPath lastTargetedPath = new PathPlannerPath();

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

  public static Command autoAlineTo(
      Target targetType, RobotContainer robot, Supplier<Translation2d> motionSupplier) {

    ProfiledPIDController angleController = DriveCommands.getAngleController();
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              target = getFlippedPose(robot.drive, targetType, motionSupplier);
              Logger.recordOutput("target", target);
              lastTargeted = target;
            }),
        Commands.defer(
            () ->
                robot.drive.pathfindToFieldPose(
                    getFlippedPose(robot.drive, targetType, motionSupplier)),
            Set.of(robot.drive)));
    // TODO: put all code on one line RIGHT after worlds
  }

  public static Command autoAlineToPath(RobotContainer robot, Branch branch) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              targetPath = pathGetter(branch);
              // Logger.recordOutput("targetPath", targetPath);
              // lastTargetedPath = targetPath;
            }),
        Commands.defer(() -> robot.drive.pathfindToPath(targetPath), Set.of(robot.drive)));
  }

  private static Command driveToTargetCommand(Drive drive, ProfiledPIDController angleController) {
    // Run the command
    return Commands.runEnd(
            () -> {
              // Update profile constraints based on calculated scalars
              TrapezoidProfile velocityProfile =
                  new TrapezoidProfile(
                      new TrapezoidProfile.Constraints(
                          DriveConstants.PF_MAX_SPEED_OR_SOMETHING * 1 * 2,
                          DriveConstants.PF_MAX_ACCEL * 1 * 2));

              // Calculate vector to target
              Translation2d toTarget =
                  new Translation2d(
                      drive.getPose().getX() - target.getX(),
                      drive.getPose().getY() - target.getY());

              // Calculate the robot's current speed towards the target
              double velocityTowardsGoal =
                  drive.getLinearSpeedMetersPerSec()
                      * dot(normalize(drive.getLinearSpeedsVector()), normalize(toTarget));

              // Grab the current drive state
              TrapezoidProfile.State state =
                  velocityProfile.calculate(
                      0.11 - 0.09,
                      new TrapezoidProfile.State(toTarget.getNorm(), velocityTowardsGoal),
                      new TrapezoidProfile.State());

              // Create a velocity vector based on the drive state's velocity
              Translation2d normalized = new Translation2d(state.velocity, toTarget.getAngle());

              // Debug info
              // SmartDashboard.putNumber("AA-Position", state.position);
              // SmartDashboard.putNumber("AA-Velocity", state.velocity);

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), target.getRotation().getRadians());

              // Check if it's red alliance
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;

              // Limit acceleration
              Translation2d finalVelocity =
                  DriveCommands.limitAccelerationFor(
                      drive.getLinearSpeedsVector(), normalized, DriveConstants.PF_MAX_ACCEL * 1);

              // Convert to field relative speeds
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      finalVelocity.getX() * (isFlipped ? -1 : 1),
                      finalVelocity.getY() * (isFlipped ? -1 : 1),
                      omega);

              // Drive the robot to targets
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            () -> target = null,
            drive)
        .beforeStarting(
            () -> {
              // Reset pid controllers
              angleController.reset(
                  drive.getRotation().getRadians(), drive.getAngularSpeedRadsPerSec());
            });
  }

  public static Pose2d getFlippedPose(
      Drive drive, Target targetType, Supplier<Translation2d> motionSupplier) {
    return flipIfRed(fromPoseId(getNewTargetPoseId(drive, targetType, motionSupplier), targetType));
  }

  /** Performs transformations to get the actual aligning vectors */
  public static int getNewTargetPoseId(
      Drive drive, Target targetType, Supplier<Translation2d> motionSupplier) {
    Pose2d pose = drive.getPose();
    Translation2d motionVector = new Translation2d();

    // Ignore transformations if no joystick values are inputted
    if (motionSupplier.get().getNorm() > 0.1) {
      boolean isFlipped =
          DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red;

      // Convert joystick inputs to robot relative (otherwise robot rotation messes with targeting)
      motionVector =
          normalize(
              motionSupplier
                  .get()
                  .rotateBy(
                      (isFlipped
                              ? pose.getRotation().plus(new Rotation2d(Math.PI))
                              : pose.getRotation())
                          .unaryMinus()));
    }

    // Find the correct pole to target
    return mapToPoseId(targetType, drive, motionVector);
  }

  /** Loops through the poses and weighs them all, returning the ID of the highest weighted pose */
  public static int getNearestPose(
      Pose2d pose, Translation2d motionVector, List<Pose2d> locations) {
    Pair<Integer, Double> bestResult = new Pair<>(0, 1000.0);

    for (int i = 0; i < locations.size(); i++) {
      // Calculate the distance from the robot to the current reef pole
      Pose2d poleLocation = flipIfRed(locations.get(i));
      double dst = pose.getTranslation().getDistance(poleLocation.getTranslation());

      // Calculate the additional weighting based on joystick angle
      double addition =
          calculateBestReefPoleAddition(poleLocation.minus(pose).getTranslation(), motionVector);

      // Apply addition and assign new best result if applicable
      double weight = dst + addition * JoystickScalar;
      if (weight <= bestResult.getSecond()) bestResult = new Pair<>(i, weight);
    }

    return bestResult.getFirst();
  }

  /**
   * Calculates a value to add to the selection "weight" of each reef pole. This is determined by
   * the dot product of the robot to reef pole vector and the vector of the joystick motion. This is
   * to ensure that the robot will prefer to target reef faces that the driver is moving towards.
   */
  public static double calculateBestReefPoleAddition(
      Translation2d toReefVector, Translation2d motionVector) {
    if (motionVector.getNorm() < 0.1) return 0.0;
    return dot(normalize(toReefVector), normalize(motionVector));
  }

  /**
   * @return A copy of the given translation vector with a magnitude of 1
   */
  public static Translation2d normalize(Translation2d vector) {
    return vector.div(vector.getNorm());
  }

  /**
   * @return The dot product of translations a and b
   */
  public static double dot(Translation2d a, Translation2d b) {
    return (a.getX() * b.getX() + a.getY() * b.getY());
  }

  /** Uses choreo utility methods to flip the given pose if on red alliance */
  public static Pose2d flipIfRed(Pose2d pose) {
    return AllianceFlipUtil.apply(pose);
  }

  public static PathPlannerPath pathGetter(Branch branch) {

    String pathName = "beans";

    switch (branch) {
      case BACK_LEFTBRANCH:
        pathName = "Back_LeftBranch";
        break;
      case BACK_RIGHTBRANCH:
        pathName = "Back_RightBranch";
        break;
      case BACKLEFT_LEFTBRANCH:
        pathName = "BackLeft_LeftBranch";
        break;
      case BACKLEFT_RIGHTBRANCH:
        pathName = "BackLeft_RightBranch";
        break;
      case BACKRIGHT_LEFTBRANCH:
        pathName = "BackRight_LeftBranch";
        break;
      case BACKRIGHT_RIGHTBRANCH:
        pathName = "BackRight_RightBranch";
        break;
      case FRONT_LEFTBRANCH:
        pathName = "Front_LeftBranch";
        break;
      case FRONT_RIGHTBRANCH:
        pathName = "Front_RightBranch";
        break;
      case FRONTLEFT_LEFTBRANCH:
        pathName = "FrontLeft_LeftBranch";
        break;
      case FRONTLEFT_RIGHTBRANCH:
        pathName = "FrontLeft_RightBranch";
        break;
      case FRONTRIGHT_LEFTBRANCH:
        pathName = "FrontRight_LeftBranch";
        break;
      case FRONTRIGHT_RIGHTBRANCH:
        pathName = "FrontRight_RightBranch";
        break;
      default:
        pathName = null;
    }

    try {
      return PathPlannerPath.fromPathFile(pathName);
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return null;
    }
  }
}
