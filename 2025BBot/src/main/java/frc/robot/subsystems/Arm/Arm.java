// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

  private ArmIO io;
  protected final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private final Alert elbowDisconnected;
  private final Alert wristDisconnected;

  
  /** Creates a new Arm. */
  public Arm(ArmIO armIO) {
    io = armIO;
    elbowDisconnected = new Alert("ELBOW DISCONNECTED", Alert.AlertType.kWarning);
    wristDisconnected = new Alert("Wrist DISCONNECTED", Alert.AlertType.kWarning);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
    elbowDisconnected.set(!inputs.elbowMotorConnected);
    wristDisconnected.set(!inputs.wristMotorConnected);
  }
}
