// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
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

  private final LoggedDashboardChooser<Command> stationChooser;
  private final SendableChooser<String> m_chooser;


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

    m_chooser = new SendableChooser<String>();
    m_chooser.addOption("Station 1", FieldConstants.Barge.closeCage);
    m_chooser.addOption("Station 2", FieldConstants.Barge.middleCage);
    m_chooser.addOption("Station 3", FieldConstants.Barge.farCage);

    stationChooser = new LoggedDashboardChooser<>("Alliance Station Number", m_chooser);
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

  public Pose2d getCagePose(){
    return m_chooser.getSelected();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
    disconnected.set(!inputs.climberMotorConnected);

    SmartDashboard.putData(m_chooser);
  }
}
