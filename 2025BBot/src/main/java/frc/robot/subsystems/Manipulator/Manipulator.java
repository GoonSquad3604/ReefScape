// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Manipulator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Manipulator extends SubsystemBase {

  private ManipulatorIO io;
  protected final ManipulatorIOInputsAutoLogged inputs = new ManipulatorIOInputsAutoLogged();
  private final Alert openingDisconnected;
  private final Alert leftWheelDisconnected;
  private final Alert rightWheelDisconnected;

  /** Creates a new Manipulator. */
  public Manipulator(ManipulatorIO manipulatorIO) {
    io = manipulatorIO;
    leftWheelDisconnected = new Alert("Left wheel manipulator motor disconnected",  Alert.AlertType.kWarning);
    rightWheelDisconnected = new Alert("Right wheel manipulator motor disconnected",  Alert.AlertType.kWarning);
    openingDisconnected = new Alert("Opening manipulator motor disconnected",  Alert.AlertType.kWarning);

  }
  public boolean hasGamePiece(){
    return io.getManipulatorSensor();
  }
  public void setOpeningToCoral(){
     io.setOpeningPos(ManipulatorConstants.coralPos);
  }
  public void setOpeningToAlgae(){
      io.setOpeningPos(ManipulatorConstants.algaePos);
  }
  public void intakeGamePiece(){
      io.setRPM(ManipulatorConstants.intakeRPM);
  }
  public void fireGamePiece(){
      io.setRPM(ManipulatorConstants.fireRPM);
  }
  public void stopWheels(){
      io.setRPM(ManipulatorConstants.zeroRPM);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Manipulator", inputs);
    leftWheelDisconnected.set(!inputs.manipulatorLeftWheelMotorConnected);
    openingDisconnected.set(!inputs.manipulatorOpeningMotorConnected);
    rightWheelDisconnected.set(!inputs.manipulatorRightWheelMotorConnected);
  }
}
