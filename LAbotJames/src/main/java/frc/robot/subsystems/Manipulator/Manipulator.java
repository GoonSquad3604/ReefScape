// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Manipulator;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LevelState;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Manipulator extends SubsystemBase {

  private ManipulatorIO io;
  protected final ManipulatorIOInputsAutoLogged inputs = new ManipulatorIOInputsAutoLogged();
  private final Alert leftWheelDisconnected;
  private final Alert rightWheelDisconnected;
  private final LoggedTunableNumber kP;
  private final LoggedTunableNumber kI;
  private final LoggedTunableNumber kD;

  /** Creates a new Manipulator. */
  public Manipulator(ManipulatorIO manipulatorIO) {
    io = manipulatorIO;

    kP = new LoggedTunableNumber("Manipulator/kP");
    kI = new LoggedTunableNumber("Manipulator/kI");
    kD = new LoggedTunableNumber("Manipulator/kD");

    kP.initDefault(0);
    kI.initDefault(0);
    kD.initDefault(0);

    leftWheelDisconnected =
        new Alert("Left wheel manipulator motor disconnected", Alert.AlertType.kWarning);
    rightWheelDisconnected =
        new Alert("Right wheel manipulator motor disconnected", Alert.AlertType.kWarning);
  }

  public boolean hasGamePiece() {
    return inputs.manipulatorPieceSensorDistance <= ManipulatorConstants.hasGamePieceThreshold;
  }

  public void stopWheels() {
    io.setWheelPower(ManipulatorConstants.zeroPower);
  }

  public void runWheelsBackwards() {
    io.setWheelPower(-0.5);
  }

  public void setWheelPower(double power) {
    io.setWheelPower(power);
  }

  public boolean isInPosition(LevelState level) {
    if (level == LevelState.L4) {
      return inputs.manipulatorReefSensorDistance <= ManipulatorConstants.l4Threshold;
    } else {
      return inputs.manipulatorReefSensorDistance <= ManipulatorConstants.reefThreshold;
    }
  }

  public Command keepAlgaeIn() {
    return runOnce(() -> io.setWheelPower(ManipulatorConstants.algaeIntakeSlow));
  }

  public Command shootAlgae(boolean isL4) {
    if (isL4) {
      return runOnce(() -> io.setWheelPower(ManipulatorConstants.bargeShoot));
    } else {
      return runOnce(() -> io.setWheelPower(ManipulatorConstants.processorShoot));
    }
  }

  public Command shootAlgaeSlower() {
    return runOnce(() -> io.setWheelPower(ManipulatorConstants.processorShoot));
  }

  public Command shootAlgaeFaster() {
    return runOnce(() -> io.setWheelPower(ManipulatorConstants.bargeShoot));
  }

  public Command shootCoral() {
    return runOnce(() -> io.setWheelPower(ManipulatorConstants.coralShoot));
  }

  public Command stopIntaking() {
    return runOnce(() -> io.setWheelPower(ManipulatorConstants.zeroPower));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Manipulator", inputs);
    leftWheelDisconnected.set(!inputs.manipulatorLeftWheelMotorConnected);
    rightWheelDisconnected.set(!inputs.manipulatorRightWheelMotorConnected);
  }
}
