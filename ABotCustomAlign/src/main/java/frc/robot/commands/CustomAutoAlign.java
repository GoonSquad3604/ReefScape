// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CustomAutoAlign extends Command {

  private static final LoggedTunableNumber drivekP = new LoggedTunableNumber("DriveToPose/DrivekP");
  private static final LoggedTunableNumber drivekD = new LoggedTunableNumber("DriveToPose/DrivekD");
  private static final LoggedTunableNumber thetakP = new LoggedTunableNumber("DriveToPose/ThetakP");
  private static final LoggedTunableNumber thetakD = new LoggedTunableNumber("DriveToPose/ThetakD");
  private static final LoggedTunableNumber driveMaxVelocity =
      new LoggedTunableNumber("DriveToPose/DriveMaxVelocity");
  private static final LoggedTunableNumber driveMaxVelocityTop =
      new LoggedTunableNumber("DriveToPose/DriveMaxVelocityTop");
  private static final LoggedTunableNumber driveMaxAcceleration =
      new LoggedTunableNumber("DriveToPose/DriveMaxAcceleration");
  private static final LoggedTunableNumber driveMaxAccelerationTop =
      new LoggedTunableNumber("DriveToPose/DriveMaxAccelerationTop");
  private static final LoggedTunableNumber driveMaxVelocityAuto =
      new LoggedTunableNumber("DriveToPose/DriveMaxVelocityAuto");
  private static final LoggedTunableNumber driveMaxVelocityAutoTop =
      new LoggedTunableNumber("DriveToPose/DriveMaxVelocityAutoTop");
  private static final LoggedTunableNumber driveMaxAccelerationAuto =
      new LoggedTunableNumber("DriveToPose/DriveMaxAccelerationAuto");
  private static final LoggedTunableNumber driveMaxAccelerationAutoTop =
      new LoggedTunableNumber("DriveToPose/DriveMaxAccelerationAutoTop");
  private static final LoggedTunableNumber thetaMaxVelocity =
      new LoggedTunableNumber("DriveToPose/ThetaMaxVelocity");
  private static final LoggedTunableNumber thetaMaxVelocityTop =
      new LoggedTunableNumber("DriveToPose/ThetaMaxVelocityTop");
  private static final LoggedTunableNumber thetaMaxAcceleration =
      new LoggedTunableNumber("DriveToPose/ThetaMaxAcceleration");
  private static final LoggedTunableNumber thetaMaxAccelerationTop =
      new LoggedTunableNumber("DriveToPose/ThetaMaxAccelerationTop");
  private static final LoggedTunableNumber thetaMaxVelocityAuto =
      new LoggedTunableNumber("DriveToPose/ThetaMaxVelocityAuto");
  private static final LoggedTunableNumber thetaMaxVelocityAutoTop =
      new LoggedTunableNumber("DriveToPose/ThetaMaxVelocityAutoTop");
  private static final LoggedTunableNumber thetaMaxAccelerationAuto =
      new LoggedTunableNumber("DriveToPose/ThetaMaxAccelerationAuto");
  private static final LoggedTunableNumber thetaMaxAccelerationAutoTop =
      new LoggedTunableNumber("DriveToPose/ThetaMaxAccelerationAutoTop");
  private static final LoggedTunableNumber elevatorMinExtension =
      new LoggedTunableNumber("DriveToPose/ElevatorMinExtension", 0.35);
  private static final LoggedTunableNumber driveTolerance =
      new LoggedTunableNumber("DriveToPose/DriveTolerance");
  private static final LoggedTunableNumber thetaTolerance =
      new LoggedTunableNumber("DriveToPose/ThetaTolerance");
  private static final LoggedTunableNumber linearFFMinRadius =
      new LoggedTunableNumber("DriveToPose/LinearFFMinRadius");
  private static final LoggedTunableNumber linearFFMaxRadius =
      new LoggedTunableNumber("DriveToPose/LinearFFMaxRadius");
  private static final LoggedTunableNumber thetaFFMinError =
      new LoggedTunableNumber("DriveToPose/ThetaFFMinError");
  private static final LoggedTunableNumber thetaFFMaxError =
      new LoggedTunableNumber("DriveToPose/ThetaFFMaxError");
  private static final LoggedTunableNumber setpointMinVelocity =
      new LoggedTunableNumber("DriveToPose/SetpointMinVelocity");
  private static final LoggedTunableNumber minDistanceVelocityCorrection =
      new LoggedTunableNumber("DriveToPose/MinDistanceVelocityCorrection");
  private static final LoggedTunableNumber minLinearErrorReset =
      new LoggedTunableNumber("DriveToPose/Reset/MinLinearError");
  private static final LoggedTunableNumber minThetaErrorReset =
      new LoggedTunableNumber("DriveToPose/Reset/MinThetaError");
  private static final LoggedTunableNumber minLinearFFSReset =
      new LoggedTunableNumber("DriveToPose/Reset/MinLinearFF");
  private static final LoggedTunableNumber minThetaFFSReset =
      new LoggedTunableNumber("DriveToPose/Reset/MinThetaFF");

  private final Drive drive;
  private final Supplier<Pose2d> target;

  public static final double loopPeriodSecs = 0.02;

  private TrapezoidProfile driveProfile;
  private final PIDController driveController = new PIDController(0.0, 0.0, 0.0, loopPeriodSecs);
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), loopPeriodSecs);

  private Translation2d lastSetpointTranslation = Translation2d.kZero;
  private Translation2d lastSetpointVelocity = Translation2d.kZero;
  private Rotation2d lastGoalRotation = Rotation2d.kZero;
  private double lastTime = 0.0;
  private double driveErrorAbs = 0.0;
  private double thetaErrorAbs = 0.0;
  private boolean running = false;
  // private Supplier<Pose2d> robot = RobotState.getInstance()::getEstimatedPose;

  private Supplier<Translation2d> linearFF = () -> Translation2d.kZero;
  private DoubleSupplier omegaFF = () -> 0.0;

  public CustomAutoAlign(Drive drive, Supplier<Pose2d> target) {
    this.drive = drive;
    this.target = target;

    // Enable continuous input for theta controller
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    if (drive != null) addRequirements(drive);
  }

  // public CustomAutoAlign(Drive drive, Supplier<Pose2d> target/*, Supplier<Pose2d> robot*/) {
  //   this(drive, target);
  //   // this.robot = robot;
  // }

  /** Creates a new CustomAutoAlign. */
  // public CustomAutoAlign(
  //     Drive drive,
  //     Supplier<Pose2d> target,
  //     Supplier<Pose2d> robot,
  //     Supplier<Translation2d> linearFF,
  //     DoubleSupplier omegaFF) {
  //       this(drive, target, robot);
  //       this.linearFF = linearFF;
  //       this.omegaFF = omegaFF;
  // }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    resetProfile();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    running = true;

    // Update from tunable numbers
    if (driveTolerance.hasChanged(hashCode())
        || thetaTolerance.hasChanged(hashCode())
        || drivekP.hasChanged(hashCode())
        || drivekD.hasChanged(hashCode())
        || thetakP.hasChanged(hashCode())
        || thetakD.hasChanged(hashCode())) {
      driveController.setP(drivekP.get());
      driveController.setD(drivekD.get());
      driveController.setTolerance(driveTolerance.get());
      thetaController.setP(thetakP.get());
      thetaController.setD(thetakD.get());
      thetaController.setTolerance(thetaTolerance.get());
    }

    // Update constraints
    // double extensionS =
    //     MathUtil.clamp(
    //         (RobotState.getInstance().getElevatorExtensionPercent() - elevatorMinExtension.get())
    //             / (1.0 - elevatorMinExtension.get()),
    //         0.0,
    //         1.0);
    // extensionS = Math.sqrt(extensionS);
    // driveProfile =
    //     new TrapezoidProfile(
    //         DriverStation.isAutonomous()
    //             ? new TrapezoidProfile.Constraints(
    //                 MathUtil.interpolate(
    //                     driveMaxVelocityAuto.get(), driveMaxVelocityAutoTop.get(), extensionS),
    //                 MathUtil.interpolate(
    //                     driveMaxAccelerationAuto.get(),
    //                     driveMaxAccelerationAutoTop.get(),
    //                     extensionS))
    //             : new TrapezoidProfile.Constraints(
    //                 MathUtil.interpolate(
    //                     driveMaxVelocity.get(), driveMaxVelocityTop.get(), extensionS),
    //                 MathUtil.interpolate(
    //                     driveMaxAcceleration.get(), driveMaxAccelerationTop.get(), extensionS)));
    // thetaController.setConstraints(
    //     new TrapezoidProfile.Constraints(
    //         DriverStation.isAutonomous()
    //             ? MathUtil.interpolate(
    //                 thetaMaxVelocityAuto.get(), thetaMaxVelocityAutoTop.get(), extensionS)
    //             : MathUtil.interpolate(
    //                 thetaMaxVelocity.get(), thetaMaxVelocityTop.get(), extensionS),
    //         DriverStation.isAutonomous()
    //             ? MathUtil.interpolate(
    //                 thetaMaxAccelerationAuto.get(), thetaMaxAccelerationAutoTop.get(),
    // extensionS)
    //             : MathUtil.interpolate(
    //                 thetaMaxAcceleration.get(), thetaMaxAccelerationTop.get()/*, extensionS*/)));

    // Get current pose and target pose
    Pose2d currentPose = drive.getPose();
    Pose2d targetPose = target.get();

    Pose2d poseError = currentPose.relativeTo(targetPose);
    driveErrorAbs = poseError.getTranslation().getNorm();
    thetaErrorAbs = Math.abs(poseError.getRotation().getRadians());
    double linearFFScaler =
        MathUtil.clamp(
            (driveErrorAbs - linearFFMinRadius.get())
                / (linearFFMaxRadius.get() - linearFFMinRadius.get()),
            0.0,
            1.0);
    double thetaFFScaler =
        MathUtil.clamp(
            (Units.radiansToDegrees(thetaErrorAbs) - thetaFFMinError.get())
                / (thetaFFMaxError.get() - thetaFFMinError.get()),
            0.0,
            1.0);

    // Calculate drive velocity
    // Calculate setpoint velocity towards target pose
    var direction = targetPose.getTranslation().minus(lastSetpointTranslation).toVector();
    double setpointVelocity =
        direction.norm()
                <= minDistanceVelocityCorrection
                    .get() // Don't calculate velocity in direction when really close
            ? lastSetpointVelocity.getNorm()
            : lastSetpointVelocity.toVector().dot(direction) / direction.norm();
    setpointVelocity = Math.max(setpointVelocity, setpointMinVelocity.get());
    State driveSetpoint =
        driveProfile.calculate(
            loopPeriodSecs,
            new State(
                direction.norm(), -setpointVelocity), // Use negative as profile has zero at target
            new State(0.0, 0.0));
    double driveVelocityScalar =
        driveController.calculate(driveErrorAbs, driveSetpoint.position)
            + driveSetpoint.velocity * linearFFScaler;
    if (driveErrorAbs < driveController.getErrorTolerance()) driveVelocityScalar = 0.0;
    Rotation2d targetToCurrentAngle =
        currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle();
    Translation2d driveVelocity = new Translation2d(driveVelocityScalar, targetToCurrentAngle);
    lastSetpointTranslation =
        new Pose2d(targetPose.getTranslation(), targetToCurrentAngle)
            .transformBy(GeomUtil.toTransform2d(driveSetpoint.position, 0.0))
            .getTranslation();
    lastSetpointVelocity = new Translation2d(driveSetpoint.velocity, targetToCurrentAngle);

    // Calculate theta speed
    double thetaSetpointVelocity =
        Math.abs((targetPose.getRotation().minus(lastGoalRotation)).getDegrees()) < 10.0
            ? (targetPose.getRotation().minus(lastGoalRotation)).getRadians()
                / (Timer.getTimestamp() - lastTime)
            : thetaController.getSetpoint().velocity;
    double thetaVelocity =
        thetaController.calculate(
                currentPose.getRotation().getRadians(),
                new State(targetPose.getRotation().getRadians(), thetaSetpointVelocity))
            + thetaController.getSetpoint().velocity * thetaFFScaler;
    if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;
    lastGoalRotation = targetPose.getRotation();
    lastTime = Timer.getTimestamp();

    // Scale feedback velocities by input ff
    final double linearS = MathUtil.clamp(linearFF.get().getNorm() * 3.0, 0.0, 1.0);
    final double thetaS = MathUtil.clamp(Math.abs(omegaFF.getAsDouble()) * 3.0, 0.0, 1.0);
    driveVelocity =
        driveVelocity.interpolate(
            linearFF.get().times(DriveConstants.PF_MAX_SPEED_OR_SOMETHING), linearS);
    thetaVelocity =
        MathUtil.interpolate(
            thetaVelocity, omegaFF.getAsDouble() * DriveConstants.PATH_MAX_ANGULAR_VELO, thetaS);
    ChassisSpeeds fieldVelocity = drive.getFieldVelocity();
    Translation2d linearFieldVelocity =
        new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);
    // Reset profiles if enough input or far enough away from setpoint
    if (linearS >= minLinearFFSReset.get()
        || thetaS >= minThetaFFSReset.get()
        || (DriverStation.isTeleop()
            && (Math.abs(driveSetpoint.position - driveErrorAbs) >= minLinearErrorReset.get()
                || Math.abs(
                        MathUtil.angleModulus(
                            currentPose.getRotation().getRadians()
                                - thetaController.getSetpoint().position))
                    >= minThetaErrorReset.get()))) {
      resetProfile();
    }

    // Command speeds
    if (drive != null) {
      drive.runVelocity(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              driveVelocity.getX(),
              driveVelocity.getY(),
              thetaVelocity,
              currentPose.getRotation()));
    }

    // Log data
    Logger.recordOutput("DriveToPose/DistanceMeasured", driveErrorAbs);
    Logger.recordOutput("DriveToPose/DistanceSetpoint", driveSetpoint.position);
    Logger.recordOutput(
        "DriveToPose/VelocityMeasured",
        -linearFieldVelocity
                .toVector()
                .dot(targetPose.getTranslation().minus(currentPose.getTranslation()).toVector())
            / driveErrorAbs);
    Logger.recordOutput("DriveToPose/VelocitySetpoint", driveSetpoint.velocity);
    Logger.recordOutput("DriveToPose/ThetaMeasured", currentPose.getRotation().getRadians());
    Logger.recordOutput("DriveToPose/ThetaSetpoint", thetaController.getSetpoint().position);
    Logger.recordOutput(
        "DriveToPose/Setpoint",
        new Pose2d[] {
          new Pose2d(
              lastSetpointTranslation,
              Rotation2d.fromRadians(thetaController.getSetpoint().position))
        });
    Logger.recordOutput("DriveToPose/Goal", new Pose2d[] {targetPose});
  }

  private void resetProfile() {
    Pose2d currentPose = drive.getPose();
    Pose2d targetPose = target.get();
    ChassisSpeeds fieldVelocity = drive.getFieldVelocity();
    Translation2d linearFieldVelocity =
        new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);

    driveProfile =
        new TrapezoidProfile(
            DriverStation.isAutonomous()
                ? new TrapezoidProfile.Constraints(
                    driveMaxVelocityAuto.get(), driveMaxAccelerationAuto.get())
                : new TrapezoidProfile.Constraints(
                    driveMaxVelocity.get(), driveMaxAcceleration.get()));

    driveController.reset();
    thetaController.reset(
        currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
    lastSetpointTranslation = currentPose.getTranslation();
    lastSetpointVelocity = linearFieldVelocity;
    lastGoalRotation = targetPose.getRotation();
    lastTime = Timer.getTimestamp();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (drive != null) drive.stop();
    running = false;
    // Clear logs
    Logger.recordOutput("DriveToPose/Setpoint", new Pose2d[] {});
    Logger.recordOutput("DriveToPose/Goal", new Pose2d[] {});
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return running
        && Math.abs(driveErrorAbs) < 1 // driveTolerance
        && Math.abs(thetaErrorAbs) < .1; // thetaTolerance.getRadians();
  }

  /** Checks if the robot pose is within the allowed drive and theta tolerances. */
  // public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
  //   return running
  //       && Math.abs(driveErrorAbs) < driveTolerance
  //       && Math.abs(thetaErrorAbs) < thetaTolerance.getRadians();
  // }
}
