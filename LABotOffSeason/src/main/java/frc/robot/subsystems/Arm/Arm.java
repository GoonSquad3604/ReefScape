// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.StateController;
import frc.robot.subsystems.SuperStructure;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {

  private ArmIO io;
  protected final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private final Alert elbowDisconnected;

  private boolean isElbowHomed = false;
  private boolean isWristHomed = false;

  private double wantedWristPose;
  private double wantedElbowPose;

  /** Creates a new Arm. */
  public Arm(ArmIO armIO) {

    io = armIO;

    elbowDisconnected = new Alert("ELBOW DISCONNECTED", Alert.AlertType.kWarning);
  }

  // public void elbowAlgaeL25() {
  //   io.setElbowPosition(ArmConstants.algaeElbowL2);
  // }

  // public void elbowAlgaeL35() {
  //   io.setElbowPosition(ArmConstants.algaeElbowL3);
  // }

  // public void wristAlgaeL25() {
  //   io.setWristPosition(ArmConstants.algaeWristL2);
  // }

  // public void wristAlgaeL35() {
  //   io.setWristPosition(ArmConstants.algaeWristL3);
  // }

  // public void elbowCoralL1() {
  //   io.setElbowPosition(ArmConstants.coralElbowL1);
  // }

  // public void elbowCoralL2() {
  //   io.setElbowPosition(ArmConstants.coralElbowL2);
  // }

  // public void elbowCoralL3() {
  //   io.setElbowPosition(ArmConstants.coralElbowL3);
  // }

  // public void elbowCoralL4() {
  //   io.setElbowPosition(ArmConstants.coralElbowL4);
  // }

  // public void wristCoralL1() {
  //   io.setWristPosition(ArmConstants.coralWristL1);
  // }

  // public void wristCoralL2() {
  //   io.setWristPosition(ArmConstants.coralWristL2);
  // }

  // public void wristCoralL3() {
  //   io.setWristPosition(ArmConstants.coralWristL3);
  // }

  // public void wristCoralL4() {
  //   io.setWristPosition(ArmConstants.coralWristL4);
  // }

  // public void source() {
  //   io.setWristPosition(ArmConstants.sourceWrist);
  //   io.setElbowPosition(ArmConstants.sourceElbow);
  // }

  public double getElbowPos() {
    return io.getElbowPosition();
  }

  public double getWristPos() {
    return io.getWristPosition();
  }

  // public void voidHome() {
  //   io.setWristPosition(ArmConstants.homeWrist);
  //   io.setElbowPosition(ArmConstants.homeElbow);
  // }

  // public Command coralL4() {
  //   return runOnce(
  //       () -> {
  //         io.setWristPosition(ArmConstants.coralWristL4);
  //         io.setElbowPosition(ArmConstants.coralElbowL4);
  //       });
  // }

  // public Command coralL3() {
  //   return runOnce(
  //       () -> {
  //         io.setWristPosition(ArmConstants.coralWristL3);
  //         io.setElbowPosition(ArmConstants.coralElbowL3);
  //       });
  // }

  // public Command coralL2() {
  //   return runOnce(
  //       () -> {
  //         io.setWristPosition(ArmConstants.coralWristL2);
  //         io.setElbowPosition(ArmConstants.coralElbowL2);
  //       });
  // }

  // public Command coralL1() {
  //   return runOnce(
  //       () -> {
  //         io.setWristPosition(ArmConstants.coralWristL1);
  //         io.setElbowPosition(ArmConstants.coralElbowL1);
  //       });
  // }

  // public void barge() {
  //   io.setWristPosition(ArmConstants.bargeWrist);
  //   io.setElbowPosition(ArmConstants.bargeElbow);
  // }

  // public void processor() {
  //   io.setWristPosition(ArmConstants.processorWrist);
  //   io.setElbowPosition(ArmConstants.processorElbow);
  // }

  // public void lollyPop() {
  //   io.setWristPosition(ArmConstants.coralWristL1);
  //   io.setElbowPosition(ArmConstants.processorElbow);
  // }

  // public void intakeFromGround() {
  //   io.setWristPosition(ArmConstants.groundWrist);
  //   io.setElbowPosition(ArmConstants.groundElbow);
  // }

  // public Command climb() {
  //   return runOnce(
  //       () -> {
  //         io.setWristPosition(ArmConstants.wristClimb);
  //         io.setElbowPosition(ArmConstants.elbowClimb);
  //       });
  // }

  // public Command home() {
  //   return runOnce(
  //       () -> {
  //         io.setWristPosition(ArmConstants.homeWrist);
  //         io.setElbowPosition(ArmConstants.homeElbow);
  //       });
  // }

  // public Command looooongIntake() {
  //   return runOnce(
  //       () -> {
  //         io.setWristPosition(ArmConstants.longSourceWrist);
  //         io.setElbowPosition(ArmConstants.longSourceElbow);
  //       });
  // }

  // public Command elbowUp() {
  //   return run(() -> io.setElbowPower(0.21));
  // }

  // public Command elbowDown() {
  //   return run(() -> io.setElbowPower(-0.21));
  // }

  // public Command wristUp() {
  //   return run(() -> io.setWristPower(0.2));
  // }

  // public Command wristDown() {
  //   return run(() -> io.setWristPower(-0.2));
  // }

  // public Command wristToPosition(double position) {
  //   return runOnce(() -> io.setWristPosition(position));
  // }

  // public Command elbowToPosition(double position) {
  //   return runOnce(() -> io.setElbowPosition(position));
  // }

  // public Command stopElbow() {
  //   return runOnce(() -> io.setElbowPower(0));
  // }

  // public Command stopWrist() {
  //   return runOnce(() -> io.setWristPower(0));
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
    elbowDisconnected.set(!inputs.elbowMotorConnected);

    Logger.recordOutput("Subsystems/Arm/ReachedSetpoint", reachedSetpoint());

    applyStates();
    
  }
  
  public void applyStates() {
    WantedState wantedState = StateController.getCurrentState();
    switch (StateController.getCurrentState()) {
      case WantedState.HOME:
        wantedElbowPose = ArmConstants.homeElbow;
        wantedWristPose = ArmConstants.homeWrist;
        break;
      case STOPPED:
        break;
      case DEFAULT_STATE:
        break;
      case INTAKE_CORAL:
        wantedElbowPose = ArmConstants.sourceElbow;
        wantedWristPose = ArmConstants.sourceWrist;
        break;
      case SCORE_L1:
      case MANUAL_L1:
        wantedElbowPose = ArmConstants.coralElbowL1;
        wantedWristPose = ArmConstants.coralWristL1;
        break;
      case SCORE_L2:
      case MANUAL_L2:
        wantedElbowPose = ArmConstants.coralElbowL2;
        wantedWristPose = ArmConstants.coralWristL2;
        break;
      case SCORE_L3:
      case MANUAL_L3:
        wantedElbowPose = ArmConstants.coralElbowL3;
        wantedWristPose = ArmConstants.coralWristL3;
        break;
      case SCORE_L4:
      case MANUAL_L4:
        wantedElbowPose = ArmConstants.coralElbowL4;
        wantedWristPose = ArmConstants.coralWristL4;
        break;
      case INTAKE_ALGAE_FROM_REEF:
        wantedElbowPose = ArmConstants.algaeReefElbow;
        wantedWristPose = ArmConstants.algaeReefWrist;
        break;
      case INTAKE_ALGAE_FROM_GROUND:
        wantedElbowPose = ArmConstants.algaeGroundElbow;
        wantedWristPose = ArmConstants.algaeGroundWrist;
        break;
      case INTAKE_ALGAE_FROM_LOLIPOP:
        wantedElbowPose = ArmConstants.algaeLollipopElbow;
        wantedWristPose = ArmConstants.algaeLollipopWrist;
        break;
      case SCORE_ALGAE_NET:
        wantedElbowPose = ArmConstants.bargeElbow;
        wantedWristPose = ArmConstants.bargeWrist;
        break;
      case SCORE_ALGAE_PROCESSOR:
        wantedElbowPose = ArmConstants.processorElbow;
        wantedWristPose = ArmConstants.processorWrist;
        break;
      case CLIMB:
        wantedElbowPose = ArmConstants.elbowClimb;
        wantedWristPose = ArmConstants.wristClimb;
        break;
    }


  }

  public boolean reachedSetpoint() {
    return MathUtil.isNear(
            wantedElbowPose,
            getElbowPos(),
            0.5)
      && MathUtil.isNear(
            wantedWristPose,
            getWristPos(),
            0.5);
  }

}
