// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  /** Creates a new Elevator. */
  public ElevatorIO io;

  protected final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final LoggedTunableNumber kP;
  private final LoggedTunableNumber kI;
  private final LoggedTunableNumber kD;

  @AutoLogOutput(key = "Elevator/atHome")
  boolean atHome = false;

  @AutoLogOutput(key = "Elevator/mahoming")
  public boolean mahoming = false;

  public Elevator(ElevatorIO elevatorIo) {

    io = elevatorIo;

    kP = new LoggedTunableNumber("Elevator/kP");
    kI = new LoggedTunableNumber("Elevator/kI");
    kD = new LoggedTunableNumber("Elevator/kD");

    kP.initDefault(0);
    kI.initDefault(0);
    kD.initDefault(0);
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

  public Command setMahoming() {
    return runOnce(() -> mahoming = true);
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
    if (mahoming && (ElevatorConstants.mahomingThreshold > io.getPos())) {

      stop();
      mahoming = false;
    }
  }
}
