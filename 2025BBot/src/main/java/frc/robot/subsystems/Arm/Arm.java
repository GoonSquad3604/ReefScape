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
    io.setElbowPosition(ArmConstants.algaeElbowL25);
  }
  public void elbowAlgaeL35(){
    io.setElbowPosition(ArmConstants.algaeElbowL35);
  }
  public void wristAlgaeL25(){
    io.setWristPosition(ArmConstants.algaeWristL25);
  }
  public void wristAlgaeL35(){
    io.setWristPosition(ArmConstants.algaeWristL35);
  }
  public void elbowCoralL1(){
    io.setElbowPosition(ArmConstants.coralElbowL1);
  }
  public void elbowCoralL2(){
    io.setElbowPosition(ArmConstants.coralElbowL2);
  }
  public void elbowCoralL3(){
    io.setElbowPosition(ArmConstants.coralElbowL3);
  }
  public void elbowCoralL4(){
    io.setElbowPosition(ArmConstants.coralElbowL4);
  }
  public void wristCoralL1(){
    io.setWristPosition(ArmConstants.coralWristL1);
  }
  public void wristCoralL2(){
    io.setWristPosition(ArmConstants.coralWristL2);
  }
  public void wristCoralL3(){
    io.setWristPosition(ArmConstants.coralWristL3);
  }
  public void wristCoralL4(){
    io.setWristPosition(ArmConstants.coralWristL4);
  }
  public void source(){
    io.setWristPosition(ArmConstants.sourceWrist); 
    io.setElbowPosition(ArmConstants.sourceElbow);
  }
  public void processor(){
    io.setWristPosition(ArmConstants.processorWrist); 
    io.setElbowPosition(ArmConstants.processorElbow);
  }
  public void barge(){
    io.setWristPosition(ArmConstants.bargeWrist);
    io.setElbowPosition(ArmConstants.bargeElbow);
  }
  public void home(){
    io.setWristPosition(ArmConstants.homeWrist); 
    io.setElbowPosition(ArmConstants.homeElbow);
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
