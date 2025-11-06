// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Manipulator.Manipulator;
import org.littletonrobotics.junction.Logger;

public class SuperStructure extends SubsystemBase {
  private Manipulator manipulator;
  private Arm arm;
  private Elevator elevator;

  public enum WantedSuperState {
    HOME,
    STOPPED,
    DEFAULT_STATE,
    INTAKE_CORAL,
    SCORE_L1,
    SCORE_L2,
    SCORE_L3,
    SCORE_L4,
    MANUAL_L4,
    MANUAL_L3,
    MANUAL_L2,
    MANUAL_L1,
    INTAKE_ALGAE_FROM_REEF,
    INTAKE_ALGAE_FROM_GROUND,
    INTAKE_ALGAE_FROM_LOLIPOP,
    SCORE_ALGAE_NET,
    SCORE_ALGAE_PROCESSOR,
    CLIMB
  }

  public enum CurrentSuperState {
    HOME,
    STOPPED,
    NO_PIECE,
    HAS_PIECE,
    INTAKE_CORAL,
    SCORE_L1,
    SCORE_L2,
    SCORE_L3,
    SCORE_L4,
    MANUAL_L4,
    MANUAL_L3,
    MANUAL_L2,
    MANUAL_L1,
    INTAKE_ALGAE_FROM_REEF,
    INTAKE_ALGAE_FROM_GROUND,
    INTAKE_ALGAE_FROM_LOLIPOP,
    SCORE_ALGAE_NET,
    SCORE_ALGAE_PROCESSOR,
    CLIMB
  }

  public static WantedSuperState wantedSuperState = WantedSuperState.STOPPED;
  private CurrentSuperState currentSuperState = CurrentSuperState.STOPPED;
  private CurrentSuperState previousSuperState;

  /** Creates a new SuperStructure. */
  public SuperStructure(Manipulator manipulator, Arm arm, Elevator elevator) {
    this.manipulator = manipulator;
    this.arm = arm;
    this.elevator = elevator;
  }

  // public Command goToL2Algae() {
  //   return runOnce(
  //       () -> {
  //         arm.elbowAlgaeL25();
  //         arm.wristAlgaeL25();
  //         manipulator.runWheels(ManipulatorConstants.algaeIntake);
  //       });
  // }

  // public Command goToL3Algae() {
  //   return runOnce(
  //       () -> {
  //         arm.elbowAlgaeL35();
  //         arm.wristAlgaeL35();
  //         manipulator.runWheels(ManipulatorConstants.algaeIntake);
  //       });
  // }

  // public Command goToSource() {
  //   return runOnce(
  //       () -> {
  //         arm.source();
  //         manipulator.runWheels(ManipulatorConstants.coralIntake);
  //       });
  // }

  // public Command goToProcessor() {
  //   return runOnce(
  //       () -> {
  //         arm.processor();
  //         manipulator.runWheels(ManipulatorConstants.algaeIntakeSlow);
  //       });
  // }

  // public Command goToLolliPop() {
  //   return runOnce(
  //       () -> {
  //         arm.lollyPop();
  //         manipulator.runWheels(ManipulatorConstants.algaeIntake);
  //       });
  // }

  // public Command intakeFromGround() {
  //   return runOnce(
  //       () -> {
  //         arm.intakeFromGround();
  //         manipulator.runWheels(ManipulatorConstants.algaeIntake);
  //       });
  // }

  // public Command goHome() {
  //   return runOnce(
  //       () -> {
  //         arm.voidHome();
  //         manipulator.stopWheels();
  //       });
  // }

  // public Command barge() {
  //   return runOnce(
  //       () -> {
  //         arm.barge();
  //         manipulator.runWheels(ManipulatorConstants.algaeIntakeSlow);
  //       });
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Superstructure/WantedSuperState", wantedSuperState);
    Logger.recordOutput("Superstructure/CurrentSuperState", currentSuperState);
    Logger.recordOutput("Superstructure/PreviousSuperState", previousSuperState);

    currentSuperState = handStateTransitions();
    applyStates();
  }

  private CurrentSuperState handStateTransitions() {
    previousSuperState = currentSuperState;
    switch (wantedSuperState) {
      default:
        currentSuperState = CurrentSuperState.STOPPED;
        break;
      case HOME:
        currentSuperState = CurrentSuperState.HOME;
        break;
      case INTAKE_CORAL:
        currentSuperState = CurrentSuperState.INTAKE_CORAL;
        break;
      case DEFAULT_STATE:
        if (manipulator.hasGamePiece()) {
          currentSuperState = CurrentSuperState.HAS_PIECE;
        } else {
          currentSuperState = CurrentSuperState.NO_PIECE;
        }
        break;
      case SCORE_L1:
        currentSuperState = CurrentSuperState.SCORE_L1;
        break;
      case SCORE_L2:
        currentSuperState = CurrentSuperState.SCORE_L2;
        break;
      case SCORE_L3:
        currentSuperState = CurrentSuperState.SCORE_L3;
        break;
      case SCORE_L4:
        currentSuperState = CurrentSuperState.SCORE_L2;
        break;
      case MANUAL_L1:
        currentSuperState = CurrentSuperState.MANUAL_L1;
        break;
      case MANUAL_L2:
        currentSuperState = CurrentSuperState.MANUAL_L2;
        break;
      case MANUAL_L3:
        currentSuperState = CurrentSuperState.MANUAL_L3;
        break;
      case MANUAL_L4:
        currentSuperState = CurrentSuperState.MANUAL_L4;
        break;
      case INTAKE_ALGAE_FROM_REEF:
        currentSuperState = CurrentSuperState.INTAKE_ALGAE_FROM_REEF;
        break;
      case INTAKE_ALGAE_FROM_GROUND:
        currentSuperState = CurrentSuperState.INTAKE_ALGAE_FROM_GROUND;
        break;
      case INTAKE_ALGAE_FROM_LOLIPOP:
        currentSuperState = CurrentSuperState.INTAKE_ALGAE_FROM_LOLIPOP;
        break;
      case SCORE_ALGAE_NET:
        currentSuperState = CurrentSuperState.SCORE_ALGAE_NET;
        break;
      case SCORE_ALGAE_PROCESSOR:
        currentSuperState = CurrentSuperState.SCORE_ALGAE_PROCESSOR;
        break;
      case CLIMB:
        currentSuperState = CurrentSuperState.CLIMB;
        break;
    }
    return currentSuperState;
  }

  private void applyStates() {
    switch (currentSuperState) {
      case HOME:
        home();
        break;
      case INTAKE_CORAL:
        intakeCoral();
        break;
      case NO_PIECE:
        noPiece();
        break;
      case HAS_PIECE:
        hasPiece();
        break;
      case SCORE_L1:
        scoreL1();
        break;
      case SCORE_L2:
        scoreL2(Constants.SuperstructureConstants.ScoringSide.LEFT);
        break;
      case SCORE_L3:
        scoreL3(Constants.SuperstructureConstants.ScoringSide.LEFT);
        break;
      case SCORE_L4:
        scoreL4(Constants.SuperstructureConstants.ScoringSide.LEFT);
        break;
      case MANUAL_L4:
        manualL4();
        break;
      case MANUAL_L3:
        manualL3();
        break;
      case MANUAL_L2:
        manualL2();
        break;
      case MANUAL_L1:
        manualL1();
        break;
      case INTAKE_ALGAE_FROM_REEF:
        intakeAlgaeFromReef();
        break;
      case INTAKE_ALGAE_FROM_GROUND:
        intakeAlgaeFromGround();
        break;
      case INTAKE_ALGAE_FROM_LOLIPOP:
        intakeAlgaeFromLolipop();
        break;
      case SCORE_ALGAE_NET:
        scoreAlgaeNet();
        break;
      case SCORE_ALGAE_PROCESSOR:
        scoreAlgaeProcessor();
        break;
      case CLIMB:
        climbDeploy();
        break;
      case STOPPED:
        stopped();
        break;
    }
  }

  private void home() {
    // ledSubsystem.setWantedAction(LEDSubsystem.WantedState.DISPLAY_ROBOT_ZERO_ACTION);

    if (arm.hasHomeCompleted() && previousSuperState == CurrentSuperState.HOME) {
      arm.setWantedState(arm.WantedState.IDLE);
    } else {
      arm.setWantedState(arm.WantedState.HOME);
    }

    if (arm.hasHomeCompleted()
          && elevator.isHome()
          && previousSuperState == CurrentSuperState.HOME) {
      setWantedSuperState(WantedSuperState.DEFAULT_STATE);
    }
  }

}
