package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public final class DriveConstants {
  // PathPlanner config constants
  public static final double ROBOT_MASS_KG = 74.088;
  public static final double ROBOT_MOI = 6.883;
  public static final double WHEEL_COF = 1.2;
  public static final double PATH_MAX_ACCEL = 3;
  public static final double PF_MAX_ACCEL = 2.0;
  public static final double PATH_MAX_ANGULAR_VELO = 540;
  public static final double PATH_MAX_ANGULAR_ACCEL = 720 / 2;
  public static final double PF_MAX_SPEED_OR_SOMETHING = 4.5050;

  public static final PPHolonomicDriveController PP_DRIVE_CONTROLLER =
      new PPHolonomicDriveController(
          new PIDConstants(5.0 * (6.0 / 5), 0.0, 0.0), new PIDConstants(5.0 * (6.0 / 5), 0.0, 0.0));
  public static final PPHolonomicDriveController PF_DRIVE_CONTROLLER =
      new PPHolonomicDriveController(
          new PIDConstants(5.0 * (6.0 / 5), 0.0, 0.1111),
          new PIDConstants(5.0 * (6.0 / 5), 0.0, 0.0)); // old d: 0.1111
}
