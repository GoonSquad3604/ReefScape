// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

  private ArmIO io;
  /** Creates a new Arm. */
  public Arm(ArmIO armIO) {
    io = armIO;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
