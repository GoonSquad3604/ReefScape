// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private ElevatorIO IO;

  public Elevator(ElevatorIO elevatorIo) {

    IO = elevatorIo;

    final Alert motorDisconnectedAlert =
        new Alert("Elevator motor disconnected!", Alert.AlertType.kWarning);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
