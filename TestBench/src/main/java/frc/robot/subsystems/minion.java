// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class minion extends SubsystemBase {
  /** Creates a new minion. */
  private SparkMax minMotor;
  private RelativeEncoder minMotorEncoder;


  public minion() {
    minMotor = new SparkMax(11, MotorType.kBrushless);
    minMotorEncoder = minMotor.getEncoder();
  }
 
  public void moveMotor(){
    minMotor.set(0.3);
  }

  public void moveMotorBackwards(){
    minMotor.set(-0.3);
  }

  public void stopMotor(){
    minMotor.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Encoder Value:", minMotorEncoder.getPosition());
  }
}