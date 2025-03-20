package frc.robot.subsystems.Climber;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class ClimberConstants {

  // Motor IDs
  public static final int motorID = 4;
  public static final int encoderID = 24;

  // Positions
  public static final double positionUp = 0.430;
  public static final double positionHome = 0.103759765625;

  // PID values
  public static final double p = 55.55; // 55555555555;
  public static final double i = 0.001;
  public static final double d = 0;
  public static final double ff = 0.0;

  // Climbing positions
  public static final Pose2d leftCagePos = new Pose2d(8.107, 7.258, Rotation2d.fromDegrees(90));
  public static final Pose2d middleCagePos = new Pose2d(8.107, 6.160, Rotation2d.fromDegrees(90));
  public static final Pose2d rightCagePos = new Pose2d(8.107, 5.075, Rotation2d.fromDegrees(90));
}
