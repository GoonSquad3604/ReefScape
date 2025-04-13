package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {
  public static final double fieldLength = Units.inchesToMeters(690.876);
  public static final double fieldWidth = Units.inchesToMeters(317);
  public static final double startingLineX =
      Units.inchesToMeters(299.438); // Measured from the inside of starting line

  public static class Processor {
    public static final Pose2d centerFace =
        new Pose2d(Units.inchesToMeters(235.726), 0, Rotation2d.fromDegrees(90));
  }

  public static class Barge {
    public static final Translation2d farCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(286.779));
    public static final Translation2d middleCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(242.855));
    public static final Translation2d closeCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(199.947));

    // Measured from floor to bottom of cage
    public static final double deepHeight = Units.inchesToMeters(3.125);
    public static final double shallowHeight = Units.inchesToMeters(30.125);

    public static final double bargeShootPosX = 7.5;
    public static final double bargeShootTheta = 0;

    public static final double bargeShootPosXOpposite = 10;
    public static final double bargeShootThetaOpposite = 180;
  }

  public static class CoralStation {
    public static final Pose2d leftCenterFace =
        new Pose2d(
            Units.inchesToMeters(33.526),
            Units.inchesToMeters(291.176),
            Rotation2d.fromDegrees(90 - 144.011));
    public static final Pose2d rightCenterFace =
        new Pose2d(
            Units.inchesToMeters(33.526),
            Units.inchesToMeters(25.824),
            Rotation2d.fromDegrees(144.011 - 90));

    public static final Pose2d rightFarIntakePos =
        new Pose2d(1.470, 0.760, Rotation2d.fromDegrees(55));
    public static final Pose2d rightNearIntakePos =
        new Pose2d(0.700, 1.315, Rotation2d.fromDegrees(55));

    public static final Pose2d leftFarIntakePos =
        new Pose2d(1.470, 7.291, Rotation2d.fromDegrees(-55));
    public static final Pose2d leftNearIntakePos =
        new Pose2d(0.700, 6.720, Rotation2d.fromDegrees(-55));
  }

  public static class Reef {
    public static final Translation2d center =
        new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.501));
    public static final double faceToZoneLine =
        Units.inchesToMeters(12); // Side of the reef to the inside of the reef zone line

    public static ArrayList<Pose2d> centerFaces = new ArrayList<>();

    // Starting facing the driver station in clockwise order
    public static final ArrayList<Map<ReefHeight, Pose3d>> branchPositions =
        new ArrayList<>(13); // Starting at the right branch facing the driver station in clockwise

    public static final ArrayList<Pose2d> rightRobotBranchPoses = new ArrayList<>(6);
    public static final ArrayList<Pose2d> leftRobotBranchPoses = new ArrayList<>(6);

    static {
      centerFaces.add(
          new Pose2d(
              Units.inchesToMeters(144.003),
              Units.inchesToMeters(158.500),
              Rotation2d.fromDegrees(180)));
      centerFaces.add(
          new Pose2d(
              Units.inchesToMeters(160.373),
              Units.inchesToMeters(186.857),
              Rotation2d.fromDegrees(120)));
      centerFaces.add(
          new Pose2d(
              Units.inchesToMeters(193.116),
              Units.inchesToMeters(186.858),
              Rotation2d.fromDegrees(60)));
      centerFaces.add(
          new Pose2d(
              Units.inchesToMeters(209.489),
              Units.inchesToMeters(158.502),
              Rotation2d.fromDegrees(0)));
      centerFaces.add(
          new Pose2d(
              Units.inchesToMeters(193.118),
              Units.inchesToMeters(130.145),
              Rotation2d.fromDegrees(-60)));
      centerFaces.add(
          new Pose2d(
              Units.inchesToMeters(160.375),
              Units.inchesToMeters(130.144),
              Rotation2d.fromDegrees(-120)));

      // Initialize branch positions
      for (int face = 0; face < 6; face++) {
        Map<ReefHeight, Pose3d> fillRight = new HashMap<>();
        Map<ReefHeight, Pose3d> fillLeft = new HashMap<>();
        Pose2d robotRight;
        Pose2d robotLeft;
        Pose2d poseDirection = new Pose2d(center, Rotation2d.fromDegrees(180 - (60 * face)));
        Pose2d poseDirectionRobot =
            new Pose2d(
                center,
                Rotation2d.fromDegrees(
                    180 - (60 * face))); // rotation of robot if it was to score at branch
        double adjustX = Units.inchesToMeters(30.738);
        double adjustY = Units.inchesToMeters(6.469);
        double robotCenterDistanceFromBranch =
            Units.inchesToMeters(
                22.50
                    * (18.5
                        / 22.5)); // placeholder - represents distance of robot center from branch
        // 30.75
        // 20 + 3.0
        // / 4 + 12
        // - 2
        double robotWhyAdjust = Units.inchesToMeters(1.5);
        for (var level : ReefHeight.values()) {

          fillRight.put(
              level,
              new Pose3d(
                  new Translation3d(
                      poseDirection
                          .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                          .getX(),
                      poseDirection
                          .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                          .getY(),
                      level.height),
                  new Rotation3d(
                      0,
                      Units.degreesToRadians(level.pitch),
                      poseDirection.getRotation().getRadians())));
          fillLeft.put(
              level,
              new Pose3d(
                  new Translation3d(
                      poseDirection
                          .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                          .getX(),
                      poseDirection
                          .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                          .getY(),
                      level.height),
                  new Rotation3d(
                      0,
                      Units.degreesToRadians(level.pitch),
                      poseDirection.getRotation().getRadians())));
        }
        robotRight =
            new Pose2d(
                new Translation2d(
                    poseDirectionRobot
                        .transformBy(
                            new Transform2d(
                                adjustX + robotCenterDistanceFromBranch,
                                adjustY - -robotWhyAdjust,
                                new Rotation2d()))
                        .getX(),
                    poseDirectionRobot
                        .transformBy(
                            new Transform2d(
                                adjustX + robotCenterDistanceFromBranch,
                                adjustY - -robotWhyAdjust,
                                new Rotation2d()))
                        .getY()),
                poseDirectionRobot.getRotation().rotateBy(Rotation2d.k180deg));
        robotLeft =
            new Pose2d(
                new Translation2d(
                    poseDirectionRobot
                        .transformBy(
                            new Transform2d(
                                adjustX + robotCenterDistanceFromBranch,
                                -adjustY - -robotWhyAdjust,
                                new Rotation2d()))
                        .getX(),
                    poseDirectionRobot
                        .transformBy(
                            new Transform2d(
                                adjustX + robotCenterDistanceFromBranch,
                                -adjustY - -robotWhyAdjust,
                                new Rotation2d()))
                        .getY()),
                poseDirectionRobot.getRotation().rotateBy(Rotation2d.k180deg));

        branchPositions.add(fillLeft);
        branchPositions.add(fillRight);
        rightRobotBranchPoses.add(robotRight);
        leftRobotBranchPoses.add(robotLeft);
      }
    }
  }

  public static class StagingPositions {
    // Measured from the center of the tree
    public static final Pose2d leftTree =
        new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(230.5), new Rotation2d());
    public static final Pose2d middleTree =
        new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(158.5), new Rotation2d());
    public static final Pose2d rightTree =
        new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(86.5), new Rotation2d());
  }

  public enum ReefHeight {
    L4(Units.inchesToMeters(72), -90),
    L3(Units.inchesToMeters(47.625), -35),
    L2(Units.inchesToMeters(31.875), -35),
    L1(Units.inchesToMeters(18), 0);

    ReefHeight(double height, double pitch) {
      this.height = height;
      this.pitch = pitch; // in degrees
    }

    public final double height;
    public final double pitch;
  }

  public static final AprilTagFieldLayout layout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
}
