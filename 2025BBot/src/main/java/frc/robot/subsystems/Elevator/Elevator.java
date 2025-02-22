// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
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
  boolean homed = false; // very false

  public Elevator(ElevatorIO elevatorIo) {

    // whee elevator go up yippee

    io = elevatorIo;

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
    io.setPower(-0.2);
  }

  public void moveDown() {
    io.setPower(0.2);
  }

  public void stop() {
    io.setPower(0);
  }

  public void homingElv() {
    io.setPos(ElevatorConstants.homePos);
    // TODO: add a homing function
  }

  public void setPosWff(double pos, double arbFF) {

    io.setPositionClosedLoopWithFF(pos, arbFF);
  }

  public double getPos() {

    return io.getPos();
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    // checks for disconnected motors
    leftDisconnected.set(!inputs.MotorLeftConnected);
    rightDisconnected.set(!inputs.MotorRightConnected);

    if (kP.hasChanged(hashCode()) || kI.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      io.setPID(kP.get(), kI.get(), kD.get());
    }

    if (!homed && io.checkLimitSwitch()) {

      io.setToZero();
      io.setPower(0);
      homed = true;

    } else if (!io.checkLimitSwitch() && homed) {

      homed = false;
    }
  }
}
