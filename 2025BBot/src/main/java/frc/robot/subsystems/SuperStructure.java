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
  private StateController stateController;

  /** Creates a new SuperStructure. */
  public SuperStructure(
      Manipulator manipulator, Arm arm, Elevator elevator, StateController stateController) {
    this.manipulator = manipulator;
    this.arm = arm;
    this.elevator = elevator;
    this.stateController = stateController;
  }

  public Command goToL1Coral() {
    return run(
        () -> {
          arm.elbowCoralL1();
          // arm.wristCoralL1();
          // elevator.elevatorCL1();
        });
  }

  public Command goToL2Coral() {
    return run(
        () -> {
          arm.elbowCoralL2();
          arm.wristCoralL2();
          stateController.setL2();
        });
  }

  public Command goToL3Coral() {
    return run(
        () -> {
          arm.elbowCoralL3();
          arm.wristCoralL3();
          stateController.setL3();
        });
  }

  public Command goToL4Coral() {
    return run(
        () -> {
          arm.elbowCoralL4();
          arm.wristCoralL4();
          elevator.elevatorCL4();
        });
  }

  public Command goToL2Algae() {
    return run(
        () -> {
          arm.elbowAlgaeL25();
          arm.wristAlgaeL25();
          // elevator.elevatorAL2();
        });
  }

  public Command goToL3Algae() {
    return run(
        () -> {
          arm.elbowAlgaeL35();
          arm.wristAlgaeL35();
          elevator.elevatorAL3();
        });
  }

  public Command goToSource() {
    return run(() -> {
          arm.source();
          elevator.source();
          manipulator.runWheels();
        })
        .andThen(intakeOff());
  }

  public Command goToProcessor() {
    return run(
        () -> {
          arm.processor();
          elevator.processor();
        });
  }

  public Command goToBarge() {
    return run(
        () -> {
          arm.barge();
          elevator.barge();
        });
  }

  public Command goHome() {
    return run(
        () -> {
          arm.home();
          // elevator.home();
        });
  }

  public Command intake() {
    return run(
        () -> {
          manipulator.intakeGamePiece();
        });
  }

  public Command setWheelCurrent() {
    return run(
        () -> {
          manipulator.wheelCurrent();
        });
  }

  public Command intakeOff() {
    return run(
        () -> {
          manipulator.stopWheels();
        });
  }

  public Command fire() {
    return run(
        () -> {
          // if (stateController.isCoralMode()) {
          manipulator.runWheelsBackwards();
          // } else {
          //   manipulator.fireAlgae();
          // }
        });
  }

  public Command moveElbowUp() {
    return run(
        () -> {
          arm.elbowUp();
        });
  }

  public Command moveElbowDown() {
    return run(
        () -> {
          arm.elbowDown();
        });
  }

  public Command moveWristUp() {
    return run(
        () -> {
          arm.wristUp();
        });
  }

  public Command moveWristDown() {
    return run(
        () -> {
          arm.wristDown();
        });
  }

  public Command moveElevatorUp() {
    return run(
        () -> {
          elevator.moveUp();
        });
  }

  public Command moveElevatorDown() {
    return run(
        () -> {
          elevator.moveDown();
        });
  }

  public Command manipulatorOpen() {
    return run(
        () -> {
          manipulator.openUp();
        });
  }

  public Command manipulatorClose() {
    return run(
        () -> {
          manipulator.close();
        });
  }

  public Command runWheels() {
    return run(
        () -> {
          manipulator.runWheels();
        });
  }

  public Command runWheelsBackwards() {
    return run(
        () -> {
          manipulator.runWheelsBackwards();
        });
  }

  public Command manipulatorStop() {
    return run(
        () -> {
          manipulator.stop();
        });
  }
  // public Command manipulatorRun() {

  public Command elbowStop() {
    return run(
        () -> {
          arm.stopElbow();
        });
  }

  public Command wristStop() {
    return run(
        () -> {
          arm.stopWrist();
        });
  }

  public Command elevatorStop() {
    return run(
        () -> {
          elevator.stop();
        });
  }

  public Command wristToPosition1() {
    return run(
        () -> {
          arm.wristCoralL1();
        });
  }

  public Command wristToPosition4() {
    return run(
        () -> {
          arm.wristCoralL4();
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
