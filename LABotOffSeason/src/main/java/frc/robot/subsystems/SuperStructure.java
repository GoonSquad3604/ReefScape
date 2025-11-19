// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmConstants;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.Manipulator.Manipulator;
import frc.robot.subsystems.Manipulator.ManipulatorConstants;
import frc.robot.util.RobotState;
import org.littletonrobotics.junction.AutoLogOutput;

public class SuperStructure extends SubsystemBase {

  private Manipulator manipulator;
  private Arm arm;
  private Elevator elevator;
  private StateController stateController;

  @AutoLogOutput private static boolean atSuperStructureSetpoints;

  /** Creates a new SuperStructure. */
  public SuperStructure(
      Manipulator manipulator, Arm arm, Elevator elevator, StateController stateController) {
    this.manipulator = manipulator;
    this.arm = arm;
    this.elevator = elevator;
    this.stateController = stateController;
    atSuperStructureSetpoints = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // get current state based on state transitions
    RobotState currentState = handleStateTransitions();

    // apply states, giving subsystems their respective positions
    applyStates(currentState);
  }

  private RobotState handleStateTransitions() {

    // get current state
    RobotState currentState = stateController.getCurrentState();

    // transition from intake coral to coral has piece
    if (currentState == RobotState.INTAKE_CORAL && manipulator.gamePieceDetected()) {
      stateController.setWantedState(RobotState.HAS_PIECE_CORAL);
    }

    // transition from intake ground or lollipop algae to processor position
    if ((currentState == RobotState.INTAKE_ALGAE_GROUND
            || currentState == RobotState.INTAKE_ALGAE_LOLIPOP)
        && manipulator.gamePieceDetected()) {
      stateController.setWantedState(RobotState.ALGAE_PROCESSOR);
    }

    // transition from intake reef algae to holding algae
    if ((currentState == RobotState.INTAKE_ALGAE_REEF_L2
            || currentState == RobotState.INTAKE_ALGAE_REEF_L3)
        && manipulator.gamePieceDetected()) {
      stateController.setWantedState(RobotState.ALGAE_PROCESSOR);
    }

    // if in any coral branch scoring mode, has piece, at setpoints (elevator + arm), and
    // manipulator sees branch AUTO FIRE!!!
    atSuperStructureSetpoints = atSuperStructureSetpoints();
    if ((currentState == RobotState.SCORE_L2
            || currentState == RobotState.SCORE_L3
            || currentState == RobotState.SCORE_L4
            || currentState == RobotState.MANUAL_L2
            || currentState == RobotState.MANUAL_L3
            || currentState == RobotState.MANUAL_L4)
        && manipulator.gamePieceDetected()
        && atSuperStructureSetpoints
        && manipulator.branchDetected(stateController.getLevel())) {
      stateController.setWantedState(RobotState.FIRE);
    }

    return stateController.getCurrentState();
  }

  private void applyStates(RobotState state) {
    switch (state) {
      case IDLE:
        idle();
        break;
      case INTAKE_CORAL:
        intakeCoral();
        break;
      case HAS_PIECE_ALGAE:
        hasPieceAlgae();
        break;
      case HAS_PIECE_CORAL:
        hasPieceCoral();
        break;
      case NO_PIECE_ALGAE:
        noPieceAlgae();
        break;
      case NO_PIECE_CORAL:
        noPieceCoral();
        break;
      case FIRE:
        fire();
        break;
        // case SCORE_L1:
        //   scoreL1();
        //   break;
      case SCORE_L2:
        scoreL2();
        break;
      case SCORE_L3:
        scoreL3();
        break;
      case SCORE_L4:
        scoreL4();
        break;
      case MANUAL_L1:
        manualL1();
        break;
      case MANUAL_L2:
        manualL2();
        break;
      case MANUAL_L3:
        manualL3();
        break;
      case MANUAL_L4:
        manualL4();
        break;
      case INTAKE_ALGAE_REEF_L2:
        intakeAlgaeReefL2();
        break;
      case INTAKE_ALGAE_REEF_L3:
        intakeAlgaeReefL3();
        break;
      case INTAKE_ALGAE_GROUND:
        intakeAlgaeGround();
        break;
      case INTAKE_ALGAE_LOLIPOP:
        intakeAlgaeLollipop();
        break;
      case ALGAE_NET:
        algaeNet();
        break;
      case ALGAE_PROCESSOR:
        algaeProcessor();
        break;
      case CLIMB:
        climbDeploy();
        break;
    }
  }

  private boolean atSuperStructureSetpoints() {
    return arm.reachedSetpoint() && elevator.reachedSetpoint();
  }

  private void idle() {}

  private void intakeCoral() {
    elevator.setElevatorPosition(ElevatorConstants.homePos);
    arm.setWantedElbowPosition(ArmConstants.sourceElbow);
    arm.setWantedWristPosition(ArmConstants.sourceWrist);
    manipulator.setWantedManipulatorPower(ManipulatorConstants.coralIntake);
  }

  private void hasPieceCoral() {
    elevator.setElevatorPosition(ElevatorConstants.homePos);
    arm.setWantedElbowPosition(ArmConstants.homeElbow);
    arm.setWantedWristPosition(ArmConstants.homeWrist);
    manipulator.setWantedManipulatorPower(0);
  }

  private void hasPieceAlgae() {
    manipulator.setWantedManipulatorPower(ManipulatorConstants.algaeIntakeSlow);
  }

  private void noPieceCoral() {
    elevator.setElevatorPosition(ElevatorConstants.homePos);
    arm.setWantedElbowPosition(ArmConstants.homeElbow);
    arm.setWantedWristPosition(ArmConstants.homeWrist);
    manipulator.setWantedManipulatorPower(0);
  }

  private void noPieceAlgae() {
    elevator.setElevatorPosition(ElevatorConstants.homePos);
    manipulator.setWantedManipulatorPower(0);
  }

  private void fire() {

    // check previous state and current level for coral, algae processor, and algae barge speeds
    RobotState previousState = stateController.getPreviousState();
    switch (previousState) {
      case ALGAE_PROCESSOR:
        // low power algae shot
        manipulator.setWantedManipulatorPower(ManipulatorConstants.processorShoot);
        break;
      case ALGAE_NET:
      case HAS_PIECE_ALGAE:
        // high power algae shot
        manipulator.setWantedManipulatorPower(ManipulatorConstants.bargeShoot);
      default:
        // vomit or fire
        manipulator.setWantedManipulatorPower(ManipulatorConstants.coralShoot);
    }
    Commands.sequence(
        Commands.waitSeconds(.3604)
            .andThen(
                Commands.runOnce(() -> stateController.setWantedState(RobotState.NO_PIECE_CORAL))));
  }

  private void scoreL2() {
    elevator.setElevatorPosition(ElevatorConstants.l2Pos);
    arm.setWantedElbowPosition(ArmConstants.coralElbowL2);
    arm.setWantedWristPosition(ArmConstants.coralWristL2);
  }

  private void scoreL3() {
    elevator.setElevatorPosition(ElevatorConstants.l3Pos);
    arm.setWantedElbowPosition(ArmConstants.coralElbowL3);
    arm.setWantedWristPosition(ArmConstants.coralWristL3);
  }

  private void scoreL4() {
    elevator.setElevatorPosition(ElevatorConstants.l4Pos);
    arm.setWantedElbowPosition(ArmConstants.coralElbowL4);
    arm.setWantedWristPosition(ArmConstants.coralWristL4);
  }

  private void manualL1() {
    elevator.setElevatorPosition(ElevatorConstants.l1Pos);
    arm.setWantedElbowPosition(ArmConstants.coralElbowL1);
    arm.setWantedWristPosition(ArmConstants.coralWristL1);
  }

  private void manualL2() {
    elevator.setElevatorPosition(ElevatorConstants.l2Pos);
    arm.setWantedElbowPosition(ArmConstants.coralElbowL2);
    arm.setWantedWristPosition(ArmConstants.coralWristL2);
  }

  private void manualL3() {
    elevator.setElevatorPosition(ElevatorConstants.l3Pos);
    arm.setWantedElbowPosition(ArmConstants.coralElbowL3);
    arm.setWantedWristPosition(ArmConstants.coralWristL3);
  }

  private void manualL4() {
    elevator.setElevatorPosition(ElevatorConstants.l4Pos);
    arm.setWantedElbowPosition(ArmConstants.coralElbowL4);
    arm.setWantedWristPosition(ArmConstants.coralWristL4);
  }

  private void intakeAlgaeReefL2() {
    elevator.setElevatorPosition(ElevatorConstants.algaeL2Pos);
    arm.setWantedElbowPosition(ArmConstants.algaeReefElbow);
    arm.setWantedWristPosition(ArmConstants.algaeReefWrist);
    manipulator.setWantedManipulatorPower(ManipulatorConstants.algaeIntake);
  }

  private void intakeAlgaeReefL3() {
    elevator.setElevatorPosition(ElevatorConstants.algaeL3Pos);
    arm.setWantedElbowPosition(ArmConstants.algaeReefElbow);
    arm.setWantedWristPosition(ArmConstants.algaeReefWrist);
    manipulator.setWantedManipulatorPower(ManipulatorConstants.algaeIntake);
  }

  private void intakeAlgaeGround() {
    elevator.setElevatorPosition(ElevatorConstants.homePos);
    arm.setWantedElbowPosition(ArmConstants.algaeGroundElbow);
    arm.setWantedWristPosition(ArmConstants.algaeGroundWrist);
    manipulator.setWantedManipulatorPower(ManipulatorConstants.algaeIntake);
  }

  private void intakeAlgaeLollipop() {
    elevator.setElevatorPosition(ElevatorConstants.homePos);
    arm.setWantedElbowPosition(ArmConstants.algaeLollipopElbow);
    arm.setWantedWristPosition(ArmConstants.algaeLollipopWrist);
    manipulator.setWantedManipulatorPower(ManipulatorConstants.algaeIntake);
  }

  private void algaeNet() {
    elevator.setElevatorPosition(ElevatorConstants.bargePos);
    arm.setWantedElbowPosition(ArmConstants.bargeElbow);
    arm.setWantedWristPosition(ArmConstants.bargeWrist);
    manipulator.setWantedManipulatorPower(ManipulatorConstants.algaeIntakeSlow);
  }

  private void algaeProcessor() {
    elevator.setElevatorPosition(ElevatorConstants.homePos);
    arm.setWantedElbowPosition(ArmConstants.processorElbow);
    arm.setWantedWristPosition(ArmConstants.processorWrist);
    manipulator.setWantedManipulatorPower(ManipulatorConstants.algaeIntakeSlow);
  }

  private void climbDeploy() {
    elevator.setElevatorPosition(ElevatorConstants.homePos);
    arm.setWantedElbowPosition(ArmConstants.elbowClimb);
    arm.setWantedWristPosition(ArmConstants.wristClimb);
    manipulator.setWantedManipulatorPower(0);
  }
}
