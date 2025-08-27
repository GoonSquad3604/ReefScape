package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.Autopilot;

public final class DriveConstants {
  // PathPlanner config constants
  public static final double ROBOT_MASS_KG = 74.088;
  public static final double ROBOT_MOI = 6.883;
  public static final double WHEEL_COF = 1.2;
  public static final double PATH_MAX_ACCEL = 3;
  public static final double PF_MAX_ACCEL = 3; // <-- the one we change when we chance auto aline
  public static final double PATH_MAX_ANGULAR_VELO = 540;
  public static final double PATH_MAX_ANGULAR_ACCEL = 720 / 2;
  public static final double PF_MAX_SPEED_OR_SOMETHING = 5;

  public static final PPHolonomicDriveController PP_DRIVE_CONTROLLER =
      new PPHolonomicDriveController(
          new PIDConstants(5.0 * (6.0 / 5), 0.0, 0.0), new PIDConstants(5.0 * (6.0 / 5), 0.0, 0.0));

  public static final PPHolonomicDriveController PF_DRIVE_CONTROLLER =
      new PPHolonomicDriveController(
          new PIDConstants(5.0 * (6.0 / 5), 0.0, 0.1111),
          new PIDConstants(5.0 * (6.0 / 5), 0.0, 0.0));

  private static final APConstraints kConstraints =
      new APConstraints().withAcceleration(5.0).withJerk(2.0);

  private static final APProfile kProfile =
      new APProfile(kConstraints)
          .withErrorXY(Centimeters.of(2))
          .withErrorTheta(Degrees.of(0.5))
          .withBeelineRadius(Centimeters.of(8));

  public static final Autopilot kAutopilot = new Autopilot(kProfile);

  public static SwerveRequest.FieldCentricFacingAngle m_request =
      new SwerveRequest.FieldCentricFacingAngle()
          .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
          .withDriveRequestType(DriveRequestType.Velocity)
          .withHeadingPID(4, 0, 0); /* change theese values for your robot */
}
