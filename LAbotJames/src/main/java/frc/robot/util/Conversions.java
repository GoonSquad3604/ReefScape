package frc.robot.util;

public class Conversions {

  /**
   * @param falconRotations Falcon rotations
   * @param gearRatio gear ratio between Falcon and mechanism
   * @return degrees of rotation of mechanism
   */
  public static double falconRotationsToMechanismDegrees(double falconRotations, double gearRatio) {
    return falconRotations * 360.0 / gearRatio;
  }

  /**
   * @param degrees Degrees of rotation of mechanism
   * @param gearRatio gear ratio between Falcon and mechanism
   * @return Falcon rotations
   */
  public static double degreesToFalconRotations(double degrees, double gearRatio) {
    return (degrees / 360.0) * gearRatio;
  }

  /**
   * @param rps Falcon rotations per second
   * @param gearRatio gear ratio between Falcon and mechanism (set to 1 for Falcon RPM)
   * @return RPM of mechanism
   */
  public static double falconRPSToMechanismRPM(double falconRPS, double gearRatio) {
    double motorRPM = falconRPS * 60.0;
    return motorRPM / gearRatio;
  }

  /**
   * @param RPM RPM of mechanism
   * @param gearRatio Gear ratio between Falcon and mechanism (set to 1 for Falcon RPS)
   * @return Falcon rotations per second
   */
  public static double rpmToFalconRPS(double rpm, double gearRatio) {
    double motorRPM = rpm * gearRatio;
    return motorRPM / 60.0;
  }

  /**
   * @param falconRotations Falcon rotations
   * @param circumference circumference of wheel in meters
   * @param gearRatio gear ratio between Falcon and mechanism
   * @return linear distance traveled by wheel in meters
   */
  public static double falconRotationsToMechanismMeters(
      double falconRotations, double circumference, double gearRatio) {
    double wheelRotations = falconRotations / gearRatio;
    return (wheelRotations * circumference);
  }

  /**
   * @param falconRPS Falcon rotations per second
   * @param circumference circumference of wheel in meters
   * @param gearRatio gear ratio between Falcon and mechanism
   * @return mechanism linear velocity in meters per second
   */
  public static double falconRPSToMechanismMPS(
      double falconRPS, double circumference, double gearRatio) {
    double wheelRPM = falconRPSToMechanismRPM(falconRPS, gearRatio);
    return (wheelRPM * circumference) / 60.0;
  }

  /**
   * @param velocity velocity in meters per second
   * @param circumference circumference of wheel in meters
   * @param gearRatio gear ratio between Falcon and mechanism
   * @return Falcon rotations per second
   */
  public static double mpsToFalconRPS(double velocity, double circumference, double gearRatio) {
    double wheelRPM = ((velocity * 60) / circumference);
    return rpmToFalconRPS(wheelRPM, gearRatio);
  }

  public static double rpmToFTPS(double rpm, double circumference) {
    return (circumference * rpm) / 60;
  }

  public static double mapRange(
      double value, double fromLow, double fromHigh, double toLow, double toHigh) {
    if (fromHigh - fromLow == 0) {
      throw new IllegalArgumentException("Input range has zero width");
    }

    return toLow + ((value - fromLow) * (toHigh - toLow)) / (fromHigh - fromLow);
  }
}
