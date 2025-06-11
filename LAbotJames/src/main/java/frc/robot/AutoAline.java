// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorToSetpoint;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.StateController.Branch;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.drive.Drive;
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
  private static Pose2d targetPose;

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
  }

  public static Command autoAlineToPose(RobotContainer robot, Branch branch) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              targetPose = poseGetter(branch);
            }),
        Commands.defer(
            () -> robot.drive.pathfindToFieldPose2(AllianceFlipUtil.apply(targetPose)),
            Set.of(robot.drive)));
  }

  public static Command autoAlineToSource(RobotContainer robot, Pose2d source) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              targetPose = source;
            }),
        Commands.defer(
            () -> robot.drive.pathfindToFieldPose2(AllianceFlipUtil.apply(targetPose)), // TODO
            Set.of(robot.drive)));
  }

  public static Command autoAlineToBarge(RobotContainer robot) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              targetPose = driveToBarge(robot);
            }),
        Commands.defer(
            () -> robot.drive.pathfindToFieldPose2(targetPose), Set.of(robot.drive))); // TODO:
  }

  public static Command autoAlineToProcessorPose(RobotContainer robot) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              targetPose = FieldConstants.Processor.robotProcessor;
            }),
        Commands.defer(
            () -> robot.drive.pathfindToFieldPose2(targetPose), Set.of(robot.drive))); // TODO:
  }

  public static Command autoAlineToAlgaeReefPose(
      RobotContainer robot, SuperStructure superStructure) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              targetPose =
                  FieldConstants.Reef.algaeReefPoses.get(robot.drive.getClosestReefPanelInt());
            }),
        Commands.defer(() -> robot.drive.pathfindToFieldPose2(targetPose), Set.of(robot.drive)),
        Commands.runOnce(
            () -> {
              targetPose =
                  FieldConstants.Reef.algaeReefPoses2.get(robot.drive.getClosestReefPanelInt());
            }),
        Commands.defer(() -> robot.drive.pathfindToFieldPose2(targetPose), Set.of(robot.drive)),
        Commands.runOnce(
            () ->
                superStructure
                    .goToProcessor()
                    .alongWith(
                        new ElevatorToSetpoint(
                            robot.elevator, ElevatorConstants.homePos, true, true))
                    .until(() -> !robot.elevator.mahoming)
                    .andThen(robot.elevator.runOnce(() -> robot.elevator.stop()))));
  }

  public static Pose2d driveToBarge(RobotContainer robot) {
    double currY = robot.drive.getPose().getY();
    double currX = robot.drive.getPose().getX();
    double newY = 0;
    double newX = 0;
    Rotation2d rot = new Rotation2d();

    if (AllianceFlipUtil.shouldFlip()
        && currY > 3) { // is red alliance and robot is on blue barge half
      newY = 3.0;
    } else if (!AllianceFlipUtil.shouldFlip()
        && currY < 5) { // is blue alliance and robot is on red barge half
      newY = 5.0;
    } else {
      newY = currY;
    }

    // check if we are on other side of field, to score from other side
    if (AllianceFlipUtil.shouldFlip()
        && currX
            < 8.5) { // if red and we are past half way of field use opposite barge x and rotation
      newX = AllianceFlipUtil.applyX(FieldConstants.Barge.bargeShootPosXOpposite);
      rot =
          AllianceFlipUtil.apply(
              Rotation2d.fromDegrees(FieldConstants.Barge.bargeShootThetaOpposite));
    } else if (!AllianceFlipUtil.shouldFlip()
        && currX > 9.5) { // if blue and we are past half way of field, use opposite barge x and
      // rotation.
      newX = AllianceFlipUtil.applyX(FieldConstants.Barge.bargeShootPosXOpposite);
      rot =
          AllianceFlipUtil.apply(
              Rotation2d.fromDegrees(FieldConstants.Barge.bargeShootThetaOpposite));
    } else {
      newX = AllianceFlipUtil.applyX(FieldConstants.Barge.bargeShootPosX);
      rot = AllianceFlipUtil.apply(Rotation2d.fromDegrees(FieldConstants.Barge.bargeShootTheta));
    }
    Pose2d targetBarge = new Pose2d(new Translation2d(newX, newY), rot);
    Logger.recordOutput("bargePose", targetBarge);
    return targetBarge;
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

  public static Pose2d poseGetter(Branch branch) {

    Pose2d thePose = new Pose2d();

    switch (branch) {
      case BACK_LEFTBRANCH:
        thePose = FieldConstants.Reef.leftRobotBranchPoses.get(3);
        break;
      case BACK_RIGHTBRANCH:
        thePose = FieldConstants.Reef.rightRobotBranchPoses.get(3);
        break;
      case BACKLEFT_LEFTBRANCH:
        thePose = FieldConstants.Reef.leftRobotBranchPoses.get(2);
        break;
      case BACKLEFT_RIGHTBRANCH:
        thePose = FieldConstants.Reef.rightRobotBranchPoses.get(2);
        break;
      case BACKRIGHT_LEFTBRANCH:
        thePose = FieldConstants.Reef.leftRobotBranchPoses.get(4);
        break;
      case BACKRIGHT_RIGHTBRANCH:
        thePose = FieldConstants.Reef.rightRobotBranchPoses.get(4);
        break;
      case FRONT_LEFTBRANCH:
        thePose = FieldConstants.Reef.leftRobotBranchPoses.get(0);
        break;
      case FRONT_RIGHTBRANCH:
        thePose = FieldConstants.Reef.rightRobotBranchPoses.get(0);
        break;
      case FRONTLEFT_LEFTBRANCH:
        thePose = FieldConstants.Reef.leftRobotBranchPoses.get(1);
        break;
      case FRONTLEFT_RIGHTBRANCH:
        thePose = FieldConstants.Reef.rightRobotBranchPoses.get(1);
        break;
      case FRONTRIGHT_LEFTBRANCH:
        thePose = FieldConstants.Reef.leftRobotBranchPoses.get(5);
        break;
      case FRONTRIGHT_RIGHTBRANCH:
        thePose = FieldConstants.Reef.rightRobotBranchPoses.get(5);
        break;
      default:
        thePose = null;
    }

    try {
      return thePose;
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return null;
    }
  }
}
