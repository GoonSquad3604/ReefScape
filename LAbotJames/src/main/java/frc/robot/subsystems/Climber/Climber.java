// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
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

  private final LoggedDashboardChooser<Pose2d> cageChooser;

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
    cageChooser.addOption("Left Cage", ClimberConstants.leftCagePos);
    cageChooser.addOption("Middle Cage", ClimberConstants.middleCagePos);
    cageChooser.addOption("Right Cage", ClimberConstants.rightCagePos);
  }

  /* Deploys the climber. */
  public Command setClimberDown() {
    return runOnce(() -> io.setPosition(ClimberConstants.positionDown));
  }

  /* Raises the climber. */
  public Command setClimberUp() {
    return runOnce(() -> io.setPosition(ClimberConstants.positionUp));
  }

  /* Sets the climber to home position */
  public Command setClimberHome() {
    return runOnce(() -> io.setPosition(ClimberConstants.positionHome));
  }

  /* Moves climber up with power */
  public Command moveClimberUp() {
    return run(() -> io.setPower(.40));
  }

  /* Moves climber down with power */
  public Command moveClimberDown() {
    return run(() -> io.setPower(-.40));
  }

  /* Stops climber */
  public Command stop() {
    return run(() -> io.setPower(0));
  }

  /* Returns the climb position based on smart dashboard option */
  public Pose2d climbPosition() {
    return cageChooser.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
    disconnected.set(!inputs.climberMotorConnected);
  }
}
