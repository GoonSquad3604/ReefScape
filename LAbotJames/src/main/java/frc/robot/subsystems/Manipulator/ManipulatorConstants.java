package frc.robot.subsystems.Manipulator;

public final class ManipulatorConstants {

  public static final int leftWheelMotorID = 10;
  public static final int rightWheelMotorID = 11;
  public static final int openingMotorID = 12;
  public static final int manipulatorSensorID = 0;
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

  public static final double openingMotorP = 1.2; // Not true value
  public static final double openingMotorI = 0.0001; // Not true value
  public static final double openingMotorD = 1.526; // Not true value
  public static final double openingMotorFF = 0; // Not true value

  public static final double wheelL1Power = 0.18050;
  public static final double wheelL2Power = 0.21;
  public static final double wheelAL2Power = -0.4000;
  public static final double wheelL3Power = 0.21;
  public static final double wheelAL3Power = -0.4000;
  public static final double wheelL4Power = 0.18;

  public static final double wheelL1RPM = 60;
  public static final double wheelL2RPM = 60;
  public static final double wheelL3RPM = 60;
  public static final double wheelL4RPM = 59;

  public static final double algaeShoot = 0.90;
  public static final double algaeIntake = -0.20;
  public static final double algaeIntakeSlow = -0.102;

  public static final double coralShoot = -0.3604;
  public static final double coralIntake = 0.5;

  public static final double reefThreshold = 500;
  public static final double l4Threshold = 455;
}
