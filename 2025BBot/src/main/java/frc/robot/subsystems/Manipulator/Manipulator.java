// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Manipulator;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Manipulator extends SubsystemBase {

  private ManipulatorIO io;
  protected final ManipulatorIOInputsAutoLogged inputs = new ManipulatorIOInputsAutoLogged();
  private final Alert openingDisconnected;
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
    openingDisconnected =
        new Alert("Opening manipulator motor disconnected", Alert.AlertType.kWarning);
  }

  public boolean hasGamePiece() {
    return io.getManipulatorSensor();
  }

  public void setOpeningToCoral() {
    io.setOpeningPos(ManipulatorConstants.coralPos);
  }

  public void setOpeningToAlgae() {
    io.setOpeningPos(ManipulatorConstants.algaePos);
  }

  public void intakeGamePiece() {
    io.setRPM(ManipulatorConstants.intakeRPM);
  }

  public void fireGamePiece() {
    io.setRPM(ManipulatorConstants.fireRPM);
  }

  public void fireCoral() {
    io.setRPM(ManipulatorConstants.coralRPM);
  }

  public void fireAlgae() {
    io.setRPM(ManipulatorConstants.algaeRPM);
  }

  public void stopWheels() {
    // io.setRPM(ManipulatorConstants.zeroRPM);
    io.setWheelPower(0);
  }

  public void openUp() {
    // io.setOpeningVoltage(1);
    io.setOpeningPower(.1);
  }

  public void close() {
    // io.setOpeningVoltage(-1);
    io.setOpeningPower(-.1);
  }

  public void stop() {
    // io.setOpeningVoltage(0);
    io.setOpeningPower(0);
  }

  public void runWheels() {
    io.setWheelPower(.4);
  }

  public void runWheelsBackwards() {
    io.setWheelPower(-.4);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Manipulator", inputs);
    leftWheelDisconnected.set(!inputs.manipulatorLeftWheelMotorConnected);
    openingDisconnected.set(!inputs.manipulatorOpeningMotorConnected);
    rightWheelDisconnected.set(!inputs.manipulatorRightWheelMotorConnected);

    // if (kP.hasChanged(hashCode()) || kI.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
    //   io.setPID(kP.get(), kI.get(), kD.get());
    // }
  }
}
