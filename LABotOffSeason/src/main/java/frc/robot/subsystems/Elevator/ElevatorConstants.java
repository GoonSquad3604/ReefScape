package frc.robot.subsystems.Elevator;

public final class ElevatorConstants {

  // motor ids
  public static final int leftMotorID = 15;
  public static final int rightMotorID = 14;

  // limit switch ids
  public static final int limitSwitch1ID = 0;
  public static final int limitSwitch2ID = 1;

  // FeedForwards
  public static final double ks = 0;
  public static final double kv = 0.002141;
  public static final double kg = 0.61331;
  public static final double ka = 0.00024162;

  // home target position
  public static final double homePos = 2;

  // threshold to disable motor power when reached
  public static final double homeThreshold = 2.75;

  // reef heights
  public static final double l1Pos = 6.0;
  public static final double l2Pos = 0;
  public static final double l3Pos = 11;
  public static final double l4Pos = 28;

  // algae heights
  public static final double algaeL2Pos = 10;
  public static final double algaeL3Pos = 17;
  public static final double bargePos = 30;
}
