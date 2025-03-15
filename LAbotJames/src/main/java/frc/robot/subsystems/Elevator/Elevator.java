// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  /** Creates a new Elevator. */
  public ElevatorIO io;

  protected final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final Alert leftDisconnected;
  private final Alert rightDisconnected;
  private final LoggedTunableNumber kP;
  private final LoggedTunableNumber kI;
  private final LoggedTunableNumber kD;
  private SysIdRoutine elevatorSysID;

  @AutoLogOutput(key = "Elevator/atHome")
  boolean atHome = false; // very false

  @AutoLogOutput(key = "Elevator/mahoming")
  public boolean mahoming = false; // falsish

  public Elevator(ElevatorIO elevatorIo) {

    // whee elevator go up yippee

    io = elevatorIo;

    elevatorSysID =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1).per(Units.Second),
                Volts.of(2),
                Time.ofBaseUnits(2.9999, Units.Second), // Use default config
                (state) -> Logger.recordOutput("Elevator/SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> this.setVolts(voltage.in(Volts)),
                null, // No log consumer, since data is recorded by AdvantageKit
                this));

    kP = new LoggedTunableNumber("Elevator/kP");
    kI = new LoggedTunableNumber("Elevator/kI");
    kD = new LoggedTunableNumber("Elevator/kD");

    kP.initDefault(0);
    kI.initDefault(0);
    kD.initDefault(0);

    // will throw an error if a motor is disconnected
    leftDisconnected = new Alert("Left elevator motor is disconnected!", Alert.AlertType.kWarning);
    rightDisconnected =
        new Alert("Right elevator motor is disconnected!", Alert.AlertType.kWarning);
  }

  // goes to a set position, C is coral and A is algae
  public void elevatorCL1() {
    io.setPos(ElevatorConstants.l1Pos);
  }

  public void elevatorCL2() {
    io.setPos(ElevatorConstants.l2Pos);
  }

  public void elevatorCL3() {
    io.setPos(ElevatorConstants.l3Pos);
  }

  public void elevatorCL4() {
    io.setPos(ElevatorConstants.l4Pos);
  }

  public void elevatorAL2() {
    io.setPos(ElevatorConstants.algaeL2Pos);
  }

  public void elevatorAL3() {
    io.setPos(ElevatorConstants.algaeL3Pos);
  }

  public void source() {
    io.setPos(ElevatorConstants.sourcePos);
  }

  public void barge() {
    io.setPos(ElevatorConstants.bargePos);
  }

  public void home() {
    io.setPos(ElevatorConstants.homePos);
  }

  public void algaeGround() {
    io.setPos(ElevatorConstants.algaeL3Pos);
  }

  public void processor() {
    io.setPos(ElevatorConstants.processorPos);
  }

  public void moveUp() {
    io.setPower(0.2);
  }

  public void moveDown() {
    io.setPower(-0.4001);
  }

  public void stop() {
    io.setPower(0);
  }

  public void setPosWff(double pos, double arbFF) {

    io.setPositionClosedLoopWithFF(pos, arbFF);
  }

  public double getPos() {

    return io.getPos();
  }

  public double getDist() {

    return io.getLCanDist();
  }

  public void setVolts(double volts) {
    io.setVoltage(volts);
  }

  public Command elevatorQuasiDown() {
    return elevatorSysID.quasistatic(SysIdRoutine.Direction.kReverse);
  }

  public Command elevatorQuasiUp() {
    return elevatorSysID.quasistatic(SysIdRoutine.Direction.kForward);
  }

  public Command elevatorDynaDown() {
    return elevatorSysID.dynamic(SysIdRoutine.Direction.kReverse);
  }

  public Command elevatorDynaUp() {
    return elevatorSysID.dynamic(SysIdRoutine.Direction.kForward);
  }

  public Command setMahoming() {
    return run(() -> mahoming = true);
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    // checks for disconnected motors
    leftDisconnected.set(!inputs.MotorLeftConnected);
    rightDisconnected.set(!inputs.MotorRightConnected);

    // if (kP.hasChanged(hashCode()) || kI.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
    //   io.setPID(kP.get(), kI.get(), kD.get());
    // }

    if (!atHome && io.checkLimitSwitch()) {

      io.setToZero();
      io.setPower(0.0);
      atHome = true;

    } else if (!io.checkLimitSwitch() && atHome) {

      atHome = false;
    }
    if (mahoming && (ElevatorConstants.mahomingThreshold > io.getPos())) {

      stop();
      mahoming = false;
    }
  }
}
