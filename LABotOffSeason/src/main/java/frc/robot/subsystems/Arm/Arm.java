// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {

  private ArmIO io;
  protected final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private final Alert elbowDisconnected;

  @AutoLogOutput private double wantedWristPosition;
  @AutoLogOutput private double wantedElbowPosition;

  /** Creates a new Arm. */
  public Arm(ArmIO armIO) {

    wantedWristPosition = ArmConstants.homeWrist;
    wantedElbowPosition = ArmConstants.homeElbow;

    io = armIO;
    elbowDisconnected = new Alert("ELBOW DISCONNECTED", Alert.AlertType.kWarning);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
    elbowDisconnected.set(!inputs.elbowMotorConnected);

    Logger.recordOutput("ReachedSetpoint", reachedSetpoint());

    setWristPosition(wantedWristPosition);
    setElbowPosition(wantedElbowPosition);
  }

  public void setWantedWristPosition(double position) {
    wantedWristPosition = position;
  }

  public void setWantedElbowPosition(double position) {
    wantedElbowPosition = position;
  }

  private void setWristPosition(double position) {
    io.setWristPosition(position);
  }

  private void setElbowPosition(double position) {
    io.setElbowPosition(position);
  }

  private double getElbowPos() {
    return io.getElbowPosition();
  }

  private double getWristPos() {
    return io.getWristPosition();
  }

  public boolean reachedSetpoint() {
    return MathUtil.isNear(wantedElbowPosition, getElbowPos(), 0.05)
        && MathUtil.isNear(wantedWristPosition, getWristPos(), 0.05);
  }
}
