// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  private ClimberIO io;
  protected final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private final Alert disconnected;

  private final LoggedTunableNumber kP;
  private final LoggedTunableNumber kI;
  private final LoggedTunableNumber kD;

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
  }

  /** Deploys the climber. * */
  public Command setClimberDown() {
    return run(() -> io.setPosition(ClimberConstants.positionDown));
  }

  /** Raises the climber. * */
  public Command setClimberUp() {
    return run(() -> io.setPosition(ClimberConstants.positionUp));
  }

  /** Sets the climber to home position */
  public Command setClimberHome() {
    return run(() -> io.setPosition(ClimberConstants.positionHome));
  }

  public Command moveClimberUp() {
    return run(() -> io.setPower(.40));
  }

  public Command moveClimberDown() {
    return run(() -> io.setPower(-.40));
  }

  public Command stop() {
    return run(() -> io.setPower(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
    disconnected.set(!inputs.climberMotorConnected);

    // if (kP.hasChanged(hashCode()) || kI.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
    //   io.setPID(kP.get(), kI.get(), kD.get());
    // }
  }
}
