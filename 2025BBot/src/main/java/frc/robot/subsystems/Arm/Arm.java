// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.StateController;

public class Arm extends SubsystemBase {

  public static Arm _instance;
  /** Creates a new Arm. */
  public Arm() {}

  public static Arm getInstance() {

        if (_instance == null) {
          _instance = new Arm();
        }
        return _instance;
      }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
