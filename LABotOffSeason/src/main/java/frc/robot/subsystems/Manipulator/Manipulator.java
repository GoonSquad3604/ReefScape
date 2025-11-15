// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Manipulator;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LevelState;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Manipulator extends SubsystemBase {

  private ManipulatorIO io;
  protected final ManipulatorIOInputsAutoLogged inputs = new ManipulatorIOInputsAutoLogged();
  private final Alert leftWheelDisconnected;
  private final Alert rightWheelDisconnected;

  @AutoLogOutput private double wantedManipulatorPower = 0;

  /** Creates a new Manipulator. */
  public Manipulator(ManipulatorIO manipulatorIO) {
    io = manipulatorIO;
    leftWheelDisconnected =
        new Alert("MANIPULATOR WHEEL_LEFT DISCONNECTED", Alert.AlertType.kWarning);
    rightWheelDisconnected =
        new Alert("MANIPULATOR WHEEL_RIGHT DISCONNECTED", Alert.AlertType.kWarning);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Manipulator", inputs);
    leftWheelDisconnected.set(!inputs.manipulatorLeftWheelMotorConnected);
    rightWheelDisconnected.set(!inputs.manipulatorRightWheelMotorConnected);

    setManipulatorPower(wantedManipulatorPower);
  }

  public void setWantedManipulatorPower(double power) {
    wantedManipulatorPower = power;
  }

  private void setManipulatorPower(double power) {
    io.setWheelPower(power);
  }

  public boolean gamePieceDetected() {
    return inputs.manipulatorGamePieceDetectionCurrentDistance
        <= ManipulatorConstants.hasGamePieceThreshold;
  }

  public boolean branchDetected(LevelState level) {
    if (level == LevelState.L4) {
      return inputs.manipulatorBranchDetectionCurrentDistance <= ManipulatorConstants.l4Threshold;
    } else {
      return inputs.manipulatorBranchDetectionCurrentDistance <= ManipulatorConstants.reefThreshold;
    }
  }
}
