// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Climber extends SubsystemBase {

  private ClimberIO io;

  protected final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private final Alert disconnected;

  private final LoggedTunableNumber kP;
  private final LoggedTunableNumber kI;
  private final LoggedTunableNumber kD;

  private final LoggedDashboardChooser<String> cageChooser;

  /** Creates a new Climber. */
  public Climber(ClimberIO io) {

    this.io = io;

    kP = new LoggedTunableNumber("Climber/kP");
    kI = new LoggedTunableNumber("Climber/kI");
    kD = new LoggedTunableNumber("Climber/kD");

    kP.initDefault(0);
    kI.initDefault(0);
    kD.initDefault(0);

    disconnected = new Alert("Climber motor disconnected!", Alert.AlertType.kWarning);

    cageChooser = new LoggedDashboardChooser<>("Cage Choices");
    // cageChooser.addOption("Left Cage", ClimberConstants.leftCagePos);
    // cageChooser.addOption("Middle Cage", ClimberConstants.middleCagePos);
    // cageChooser.addOption("Right Cage", ClimberConstants.rightCagePos);

    cageChooser.addOption("Left Cage", "LeftClimber");
    cageChooser.addOption("Middle Cage", "MiddleClimber");
    cageChooser.addOption("Right Cage", "RightClimber");
  }

  /* Raises the climber. */
  public Command setClimberUp() {
    return runOnce(() -> io.setPosition(ClimberConstants.positionUp));
  }

  /* Sets the climber to home position */
  public Command setClimberHome() {
    return runOnce(() -> io.setPosition(ClimberConstants.positionHome));
  }

  /* Moves climber up with power (raise) */
  public Command moveClimberUp() {
    return run(() -> io.setPower(.9));
  }

  /* Moves climber down with power (deploy) */
  public Command moveClimberDown() {
    return run(() -> io.setPower(-.9));
  }

  public Command setPower(double power) {
    return run(() -> io.setPower(power));
  }

  /* Stops climber */
  public Command stop() {
    return run(() -> io.setPower(0));
  }

  /* Returns the climb position based on smart dashboard option */
  public PathPlannerPath getClimbPath() {
    try {
      return PathPlannerPath.fromPathFile(cageChooser.get());
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return null;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
    disconnected.set(!inputs.climberMotorConnected);
  }
}
