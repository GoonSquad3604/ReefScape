// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Manipulator.Manipulator;
import frc.robot.subsystems.Manipulator.ManipulatorConstants;

public class SuperStructure extends SubsystemBase {
  private Manipulator manipulator;
  private Arm arm;
  private Elevator elevator;

  /** Creates a new SuperStructure. */
  public SuperStructure(Manipulator manipulator, Arm arm, Elevator elevator) {
    this.manipulator = manipulator;
    this.arm = arm;
    this.elevator = elevator;
  }

  public Command goToL2Algae() {
    return runOnce(
        () -> {
          arm.elbowAlgaeL25();
          arm.wristAlgaeL25();
          manipulator.runWheels(ManipulatorConstants.algaeIntake);
        });
  }

  public Command goToL3Algae() {
    return runOnce(
        () -> {
          arm.elbowAlgaeL35();
          arm.wristAlgaeL35();
          manipulator.runWheels(ManipulatorConstants.algaeIntake);
        });
  }

  public Command goToSource() {
    return runOnce(
        () -> {
          arm.source();
          manipulator.runWheels(ManipulatorConstants.coralIntake);
        });
  }

  public Command goToProcessor() {
    return runOnce(
        () -> {
          arm.processor();
          manipulator.runWheels(ManipulatorConstants.algaeIntakeSlow);
        });
  }

  public Command goToLolliPop() {
    return runOnce(
        () -> {
          arm.lollyPop();
          manipulator.runWheels(ManipulatorConstants.algaeIntake);
        });
  }

  public Command intakeFromGround() {
    return runOnce(
        () -> {
          arm.intakeFromGround();
          manipulator.runWheels(ManipulatorConstants.algaeIntake);
        });
  }

  public Command goHome() {
    return runOnce(
        () -> {
          arm.voidHome();
          manipulator.stopWheels();
        });
  }

  public Command barge() {
    return runOnce(
        () -> {
          arm.barge();
          manipulator.runWheels(ManipulatorConstants.algaeIntakeSlow);
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
