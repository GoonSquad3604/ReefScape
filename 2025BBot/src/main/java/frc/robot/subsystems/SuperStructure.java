// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Manipulator.Manipulator;

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

  public Command goToL1Coral(){
    return run(() -> {
      arm.elbowCoralL1();
      arm.wristCoralL1();
      elevator.elevatorCL1();
    });
  }
  public Command goToL2Coral(){
    return run(() -> {
      arm.elbowCoralL2();
      arm.wristCoralL2();
      elevator.elevatorCL2();
    });
  }
  public Command goToL3Coral(){
    return run(() -> {
      arm.elbowCoralL3();
      arm.wristCoralL3();
      elevator.elevatorCL3();
    });
  }
  public Command goToL4Coral(){
    return run(() -> {
      arm.elbowCoralL4();
      arm.wristCoralL4();
      elevator.elevatorCL4();
    });
  }
  public Command goToL2Algae(){
    return run(() -> {
      arm.elbowAlgaeL25();
      arm.wristAlgaeL25();
      elevator.elevatorAL2();
    });
  }
  public Command goToL3Algae(){
    return run(() -> {
      arm.elbowAlgaeL35();
      arm.wristAlgaeL35();
      elevator.elevatorAL3();
    });
  }
  public Command goToSource(){
    return run(() -> {
      arm.source();
      elevator.source();
    });
  }
  public Command goToProcessor(){
    return run(() -> {
      arm.processor();
      elevator.processor();
    });
  }
  public Command goToBarge(){
    return run(() -> {
      arm.barge();
      elevator.barge();
    });
  }
  public Command intake(){
    return run(() -> {
      arm.intake();
      manipulator.intakeGamePiece();
    });
  }
  // public Command intakeOff(){
  //   return run(() -> {
  //     arm.();
  //     manipulator.intakeGamePiece();
  //   });
  // }
  public Command fire(){
    return run(() -> manipulator.fireGamePiece());
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
