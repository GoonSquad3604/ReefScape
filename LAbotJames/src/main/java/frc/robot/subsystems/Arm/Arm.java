// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {

  private ArmIO io;
  protected final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private final Alert elbowDisconnected;
  private final Alert wristDisconnected;

  private final LoggedTunableNumber kP;
  private final LoggedTunableNumber kI;
  private final LoggedTunableNumber kD;

  private SysIdRoutine elbowSysID;
  private SysIdRoutine wristSysID;

  /** Creates a new Arm. */
  public Arm(ArmIO armIO) {

    elbowSysID =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null, // Use default config
                (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> this.elbowVoltage(voltage.in(Volts)),
                null, // No log consumer, since data is recorded by AdvantageKit
                this));

    wristSysID =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null, // Use default config
                (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> this.wristVoltage(voltage.in(Volts)),
                null, // No log consumer, since data is recorded by AdvantageKit
                this));

    kP = new LoggedTunableNumber("Arm/kP");
    kI = new LoggedTunableNumber("Arm/kI");
    kD = new LoggedTunableNumber("Arm/kD");

    kP.initDefault(0);
    kI.initDefault(0);
    kD.initDefault(0);

    io = armIO;
    elbowDisconnected = new Alert("ELBOW DISCONNECTED", Alert.AlertType.kWarning);
    wristDisconnected = new Alert("Wrist DISCONNECTED", Alert.AlertType.kWarning);
  }

  public void elbowAlgaeL25() {
    io.setElbowPosition(ArmConstants.algaeElbowL25);
  }

  public void elbowAlgaeL35() {
    io.setElbowPosition(ArmConstants.algaeElbowL35);
  }

  public void wristAlgaeL25() {
    io.setWristPosition(ArmConstants.algaeWristL25);
  }

  public void wristAlgaeL35() {
    io.setWristPosition(ArmConstants.algaeWristL35);
  }

  public void elbowCoralL1() {
    io.setElbowPosition(ArmConstants.coralElbowL1);
  }

  public void elbowCoralL2() {
    io.setElbowPosition(ArmConstants.coralElbowL2);
  }

  public void elbowCoralL3() {
    io.setElbowPosition(ArmConstants.coralElbowL3);
  }

  public void elbowCoralL4() {
    io.setElbowPosition(ArmConstants.coralElbowL4);
  }

  public void wristCoralL1() {
    io.setWristPosition(ArmConstants.coralWristL1);
  }

  public void wristCoralL2() {
    io.setWristPosition(ArmConstants.coralWristL2);
  }

  public void wristCoralL3() {
    io.setWristPosition(ArmConstants.coralWristL3);
  }

  public void wristCoralL4() {
    io.setWristPosition(ArmConstants.coralWristL4);
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

  // public void processor() {
  //   io.setWristPosition(ArmConstants.processorWrist);
  //   io.setElbowPosition(ArmConstants.processorElbow);
  // }

  // public void barge() {
  //   io.setWristPosition(ArmConstants.bargeWrist);
  //   io.setElbowPosition(ArmConstants.bargeElbow);
  // }

  public void voidHome() {
    io.setWristPosition(ArmConstants.homeWrist);
    io.setElbowPosition(ArmConstants.homeElbow);
  }

  // public void climb() {
  //   io.setWristPosition(ArmConstants.wristClimb);
  //   io.setElbowPosition(ArmConstants.elbowClimb);
  // }

  // public void layup() {
  //   io.setWristPosition(ArmConstants.alleyOop);
  // }

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

  public void barge() {

    io.setWristPosition(ArmConstants.bargeWrist);
    io.setElbowPosition(ArmConstants.bargeElbow);
    ;
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

  public Command layup() {
    return runOnce(
        () -> {
          io.setWristPosition(ArmConstants.alleyOop);
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
    // io.setWristMotorVoltage(1);
  }

  public Command elbowDown() {
    return run(() -> io.setElbowPower(-0.21));
    // io.setWristMotorVoltage(-1);
  }

  public Command wristUp() {
    // io.setElbowPower(0.21);
    return run(() -> io.setWristPower(0.1999999999999999998999999999999999999));
  }

  public Command wristDown() {
    // io.setElbowPower(-0.21);
    return run(() -> io.setWristPower(-0.1999999999999999999999999));
  }

  public Command wristToPosition(double position) {
    return runOnce(() -> io.setWristPosition(position));
  }

  public Command elbowToPosition(double position) {
    return runOnce(() -> io.setElbowPosition(position));
  }

  public Command stopElbow() {
    return runOnce(() -> io.setElbowPower(0));
    // io.setWristMotorVoltage(0);
  }

  public Command stopWrist() {
    // io.setElbowPower(0);
    return runOnce(() -> io.setWristPower(0));
  }

  public void elbowVoltage(double volts) {
    io.setElbowMotorVoltage(volts);
  }

  public void wristVoltage(double volts) {
    io.setWristMotorVoltage(volts);
  }

  public Command elbowQuasiForward() {
    return elbowSysID.quasistatic(SysIdRoutine.Direction.kForward);
  }

  public Command elbowQuasiBackward() {
    return elbowSysID.quasistatic(SysIdRoutine.Direction.kReverse);
  }

  public Command elbowDynaForward() {
    return elbowSysID.dynamic(SysIdRoutine.Direction.kForward);
  }

  public Command elbowDynaBackward() {
    return elbowSysID.dynamic(SysIdRoutine.Direction.kReverse);
  }

  public Command wristQuasiForward() {
    return elbowSysID.quasistatic(SysIdRoutine.Direction.kForward);
  }

  public Command wristQuasiBackward() {
    return elbowSysID.quasistatic(SysIdRoutine.Direction.kReverse);
  }

  public Command wristDynaForward() {
    return elbowSysID.dynamic(SysIdRoutine.Direction.kForward);
  }

  public Command wristDynaBackward() {
    return elbowSysID.dynamic(SysIdRoutine.Direction.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
    elbowDisconnected.set(!inputs.elbowMotorConnected);
    wristDisconnected.set(!inputs.wristMotorConnected);

    // if (kP.hasChanged(hashCode()) || kI.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
    //   io.setPID(kP.get(), kI.get(), kD.get());
    // }
  }
}
