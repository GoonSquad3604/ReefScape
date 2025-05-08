// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  private ClimberIO io;

  protected final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private final Alert disconnected;



  /** Creates a new Climber. */
  public Climber(ClimberIO io) {

    this.io = io;

    disconnected = new Alert("Climber motor disconnected!", Alert.AlertType.kWarning);

  }


  /* Moves climber up with power (raise) */
  public Command moveClimberUp() {
    return run(() -> io.setPower(.9));
  }

  /* Moves climber down with power (deploy) */
  public Command moveClimberDown() {
    return run(() -> io.setPower(-.9));
  }

  /* Sets climber motor to the given power */
  public Command setPower(double power) {
    return run(() -> io.setPower(power));
  }

  /* Stops climber */
  public Command stop() {
    return run(() -> io.setPower(0));
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
    disconnected.set(!inputs.climberMotorConnected);
  }
}
