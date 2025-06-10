package frc.robot.subsystems.Manipulator;

public final class ManipulatorConstants {

  public static final int leftWheelMotorID = 10;
  public static final int rightWheelMotorID = 11;
  public static final int manipulatorSensorID = 49;
  public static final int manipulatorDistanceSensorID = 42;

  public static final double coralPos = 0.67;
  public static final double algaePos = 0.574;

  public static final double wheelPower = .2;
  public static final double backwardsWheelPower = -1.0 / 3;
  public static final double intakeRPM = -2;
  public static final double fireRPM = 2;
  public static final double coralRPM = 5;
  public static final double algaeRPM = 3;
  public static final double zeroPower = 0;
  public static final double wheelP = 1; // Not true value
  public static final double wheelI = 0; // Not true value
  public static final double wheelD = 0; // Not true value
  public static final double wheelFF = 0; // Not true value

  public static final double bargeShoot = .9;
  public static final double processorShoot = 0.3;
  public static final double algaeIntake = -0.5;
  public static final double algaeIntakeSlow = -0.10;

  public static final double coralShoot = -0.3604;
  public static final double coralIntake = 0.5;

  public static final double reefThreshold = 500;
  public static final double l4Threshold = 455;
  public static final double hasGamePieceThreshold = 125;

  public static final double leftToRightRatio = -0.8;
}
