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
    return runOnce(
        () -> {
          arm.elbowCoralL1();
          arm.wristCoralL1();
          // elevator.elevatorCL1();
        });
  }

  public Command goToL2Coral() {
    return runOnce(
        () -> {
          arm.elbowCoralL2();
          arm.wristCoralL2();
          stateController.setL2();
        });
  }

  public Command goToL3Coral() {
    return runOnce(
        () -> {
          arm.elbowCoralL3();
          arm.wristCoralL3();
          stateController.setL3();
        });
  }

  public Command goToL4Coral() {
    return runOnce(
        () -> {
          arm.elbowCoralL4();
          arm.wristCoralL4();
        });
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

  public Command goToBarge() {
    return runOnce(
        () -> {
          arm.barge();
          elevator.barge();
        });
  }

  public Command goHome() {
    return runOnce(
        () -> {
          arm.voidHome();
          manipulator.stopWheels();
        });
  }

  public Command intake() {
    return runOnce(
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
    return runOnce(
        () -> {
          manipulator.stopWheels();
        });
  }

  public Command fire() {
    return runOnce(
        () -> {
          switch (stateController.getLevel()) {
            case L1:
              manipulator.runWheels(-ManipulatorConstants.wheelL1Power);
              break;
            case L2:
              manipulator.runWheels(-ManipulatorConstants.wheelL2Power);
              break;
            case L3:
              manipulator.runWheels(-ManipulatorConstants.wheelL3Power);
              break;
            case L4:
              manipulator.runWheels(-ManipulatorConstants.wheelL4Power);
              break;
            default:
              manipulator.runWheels(0);
          }
        });
  }

  public Command barge() {
    return runOnce(
        () -> {
          arm.barge();
          // manipulator.runWheels(ManipulatorConstants.algaeIntake);
        });
  }

  // public Command vomit() {
  //   return run(
  //       () -> {
  //         manipulator.runWheels(ManipulatorConstants.backwardsWheelPower);
  //       });
  // }

  // public Command moveElbowUp() {
  //   return run(
  //       () -> {
  //         arm.elbowUp();
  //       });
  // }

  // public Command moveElbowDown() {
  //   return run(
  //       () -> {
  //         arm.elbowDown();
  //       });
  // }

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
    return runOnce(
        () -> {
          elevator.moveUp();
        });
  }

  public Command moveElevatorDown() {
    return runOnce(
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
    return runOnce(
        () -> {
          arm.stopElbow();
        });
  }

  public Command wristStop() {
    return runOnce(
        () -> {
          arm.stopWrist();
        });
  }

  public Command elevatorStop() {
    return runOnce(
        () -> {
          elevator.stop();
        });
  }

  public Command armClimb() {
    return runOnce(
        () -> {
          arm.climb();
        });
  }

  public Command layup() {
    return run(
        () -> {
          arm.layup();
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
