// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.StateController;

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

  public void elbowAlgaeL25(){
    run(() -> io.setElbowPosition(ArmConstants.algaeElbowL25));
  }
  public void elbowAlgaeL35(){
    run(() -> io.setElbowPosition(ArmConstants.algaeElbowL35));
  }
  public void wristAlgaeL25(){
    run(() -> io.setWristPosition(ArmConstants.algaeWristL25));
  }
  public void wristAlgaeL35(){
    run(() -> io.setWristPosition(ArmConstants.algaeWristL35));
  }
  public void elbowCoralL1(){
    run(() -> io.setElbowPosition(ArmConstants.coralElbowL1));
  }
  public void elbowCoralL2(){
    run(() -> io.setElbowPosition(ArmConstants.coralElbowL2));
  }
  public void elbowCoralL3(){
    run(() -> io.setElbowPosition(ArmConstants.coralElbowL3));
  }
  public void elbowCoralL4(){
    run(() -> io.setElbowPosition(ArmConstants.coralElbowL4));
  }
  public void wristCoralL1(){
    run(() -> io.setWristPosition(ArmConstants.coralWristL1));
  }
  public void wristCoralL2(){
    run(() -> io.setWristPosition(ArmConstants.coralWristL2));
  }
  public void wristCoralL3(){
    run(() -> io.setWristPosition(ArmConstants.coralWristL3));
  }
  public void wristCoralL4(){
    run(() -> io.setWristPosition(ArmConstants.coralWristL4));
  }
  public void source(){
    run(() -> {io.setWristPosition(ArmConstants.sourceWrist); io.setElbowPosition(ArmConstants.sourceElbow);});
  }
  public void processor(){
    run(() -> {io.setWristPosition(ArmConstants.processorWrist); io.setElbowPosition(ArmConstants.processorElbow);});
  }
  public void barge(){
    run(() -> {io.setWristPosition(ArmConstants.bargeWrist); io.setElbowPosition(ArmConstants.bargeElbow);});
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
