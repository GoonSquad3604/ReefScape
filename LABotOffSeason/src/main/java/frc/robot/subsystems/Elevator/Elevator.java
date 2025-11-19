// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ElevatorToSetpoint;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  public ElevatorIO io;

  protected final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  @AutoLogOutput(key = "Elevator/atHome")
  private boolean atHome = false;

  @AutoLogOutput(key = "Elevator/mahoming")
  private boolean mahoming = false;

  @AutoLogOutput private double wantedElevatorPosition;

  /** Creates a new Elevator. */
  public Elevator(ElevatorIO elevatorIo) {
    io = elevatorIo;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    if (!atHome && io.checkLimitSwitch()) {

      io.setToZero();
      io.setPower(0.0);
      atHome = true;

    } else if (!io.checkLimitSwitch() && atHome) {

      atHome = false;
    }
    if (mahoming && (ElevatorConstants.homeThreshold > io.getPos())) {

      stop();
      mahoming = false;
    }
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

  public Command setMahoming() {
    return runOnce(() -> mahoming = true);
  }

  public boolean isHome() {
    return atHome;
  }

  public void setElevatorPosition(double wantedPosition) {
    wantedElevatorPosition = wantedPosition;
    new ElevatorToSetpoint(this, wantedPosition);
  }

  public boolean reachedSetpoint() {
    return MathUtil.isNear(wantedElevatorPosition, getPos(), 0.25);
  }
}
