package frc.robot.subsystems.Manipulator;

import org.littletonrobotics.junction.AutoLog;

public interface ManipulatorIO {

  default void updateInputs(ManipulatorIOInputs inputs) {}

  @AutoLog
  class ManipulatorIOInputs {

    public boolean manipulatorLeftWheelMotorConnected = false;
    public boolean manipulatorRightWheelMotorConnected = false;

    public boolean manipulatorGamePieceDetected = false;
    public double manipulatorGamePieceDetectionCurrentDistance;

    public double manipulatorBranchDetectionCurrentDistance;

    public double manipulatorLeftWheelMotorVoltage;
    public double manipulatorRightWheelMotorVoltage;

    public double manipulatorLeftWheelMotorCurrent;
    public double manipulatorLeftWheelStatorCurrent;
    public double manipulatorRightWheelMotorCurrent;

    public double manipulatorRightRPM;
    public double manipulatorLeftRPM;

    public double manipulatorRightWheelTemperature;
    public double manipulatorLeftWheelTemperature;
  }

  default void setWheelPower(double power) {}
}
