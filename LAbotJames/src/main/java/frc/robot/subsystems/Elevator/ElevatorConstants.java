package frc.robot.subsystems.Elevator;

public final class ElevatorConstants {

  // motor ids
  public static final int leftMotorID = 15;
  public static final int rightMotorID = 14;

  // limit switch ids
  public static final int limitSwitch1ID = 0;
  public static final int limitSwitch2ID = 1;

  // PIDs
  public static final double elevatorP = 0.155001;
  public static final double elevatorI = 0;
  public static final double elevatorD = 0.0012983;
  public static final double elvatorff = 0;

  // FeedForwards
  public static final double ks = 0;
  public static final double kv = 0.002141;
  public static final double kg = 0.61331;
  public static final double ka = 0.00024162;

  // default home position
  public static final double homePos = 2;

  // each coral branch height
  public static final double l1Pos = 6.0;
  public static final double l2Pos = 0;
  public static final double l3Pos = 11;
  public static final double l4Pos = 28;

  // each coral branch height (ALGAE REMOVAL) DO NOT USE LUCAS MESSED UP
  public static final double l2PosAlgae = 16;
  public static final double l3PosAlgae = 25;

  // algae on reef heights
  public static final double algaeL2Pos = 10;
  public static final double algaeL3Pos = 17;

  // misc position
  public static final double sourcePos = 0;
  public static final double bargePos = 30;
  public static final double processorPos = 0;
  public static final double groundAlgaeIntake = 0;
  public static final double lolipopAlgaeIntake = 0;

  public static final double mahomingThreshold = 2.75;

  // the offset
  public static final double homeOffset = 0;
}
