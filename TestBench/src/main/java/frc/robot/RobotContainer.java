// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.minion;

public class RobotContainer {

  private CommandXboxController driver = new CommandXboxController(0);
  private minion minion = new minion();
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    driver.a().onTrue(new InstantCommand(()-> minion.moveMotor() ));
    driver.a().onFalse(new InstantCommand(()-> minion.stopMotor() ));

    driver.b().onTrue(new InstantCommand(()-> minion.moveMotorBackwards() ));
    driver.b().onFalse(new InstantCommand(()-> minion.stopMotor() ));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
