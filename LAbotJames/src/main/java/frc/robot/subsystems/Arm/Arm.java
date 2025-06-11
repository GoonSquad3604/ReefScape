// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {

  private ArmIO io;
  protected final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private final Alert elbowDisconnected;

  private final LoggedTunableNumber kP;
  private final LoggedTunableNumber kI;
  private final LoggedTunableNumber kD;

  /** Creates a new Arm. */
  public Arm(ArmIO armIO) {

    kP = new LoggedTunableNumber("Arm/kP");
    kI = new LoggedTunableNumber("Arm/kI");
    kD = new LoggedTunableNumber("Arm/kD");

    kP.initDefault(0);
    kI.initDefault(0);
    kD.initDefault(0);

    io = armIO;
    elbowDisconnected = new Alert("ELBOW DISCONNECTED", Alert.AlertType.kWarning);
  }

  public void source() {
    io.setWristPosition(ArmConstants.sourceWrist);
    io.setElbowPosition(ArmConstants.sourceElbow);
  }

  public double getElbowPos() {
    return io.getElbowPosition();
  }

  public double getWristPos() {
    return io.getWristPosition();
  }

  public void voidHome() {
    io.setWristPosition(ArmConstants.homeWrist);
    io.setElbowPosition(ArmConstants.homeElbow);
  }

  public Command coralL4() {
    return runOnce(
        () -> {
          io.setWristPosition(ArmConstants.coralWristL4);
          io.setElbowPosition(ArmConstants.coralElbowL4);
        });
  }

  public Command coralL3() {
    return runOnce(
        () -> {
          io.setWristPosition(ArmConstants.coralWristL3);
          io.setElbowPosition(ArmConstants.coralElbowL3);
        });
  }

  public Command coralL2() {
    return runOnce(
        () -> {
          io.setWristPosition(ArmConstants.coralWristL2);
          io.setElbowPosition(ArmConstants.coralElbowL2);
        });
  }

  public Command coralL1() {
    return runOnce(
        () -> {
          io.setWristPosition(ArmConstants.coralWristL1);
          io.setElbowPosition(ArmConstants.coralElbowL1);
        });
  }

  public void algaeL2() {
    io.setWristPosition(ArmConstants.algaeWristL2);
    io.setElbowPosition(ArmConstants.algaeWristL2);
  }

  public void algaeL3() {
    io.setWristPosition(ArmConstants.algaeWristL3);
    io.setElbowPosition(ArmConstants.algaeWristL3);
  }

  public void barge() {
    io.setWristPosition(ArmConstants.bargeWrist);
    io.setElbowPosition(ArmConstants.bargeElbow);
  }

  public void processor() {
    io.setWristPosition(ArmConstants.processorWrist);
    io.setElbowPosition(ArmConstants.processorElbow);
  }

  public void lollyPop() {
    io.setWristPosition(ArmConstants.coralWristL1);
    io.setElbowPosition(ArmConstants.processorElbow);
  }

  public void intakeFromGround() {
    io.setWristPosition(ArmConstants.groundWrist);
    io.setElbowPosition(ArmConstants.groundElbow);
  }

  public Command climb() {
    return runOnce(
        () -> {
          io.setWristPosition(ArmConstants.wristClimb);
          io.setElbowPosition(ArmConstants.elbowClimb);
        });
  }

  public Command home() {
    return runOnce(
        () -> {
          io.setWristPosition(ArmConstants.homeWrist);
          io.setElbowPosition(ArmConstants.homeElbow);
        });
  }

  public Command looooongIntake() {
    return runOnce(
        () -> {
          io.setWristPosition(ArmConstants.longSourceWrist);
          io.setElbowPosition(ArmConstants.longSourceElbow);
        });
  }

  public Command elbowUp() {
    return run(() -> io.setElbowPower(0.21));
  }

  public Command elbowDown() {
    return run(() -> io.setElbowPower(-0.21));
  }

  public Command wristUp() {
    return run(() -> io.setWristPower(0.2));
  }

  public Command wristDown() {
    return run(() -> io.setWristPower(-0.2));
  }

  public Command stopElbow() {
    return runOnce(() -> io.setElbowPower(0));
  }

  public Command stopWrist() {
    return runOnce(() -> io.setWristPower(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
    elbowDisconnected.set(!inputs.elbowMotorConnected);
  }
}
