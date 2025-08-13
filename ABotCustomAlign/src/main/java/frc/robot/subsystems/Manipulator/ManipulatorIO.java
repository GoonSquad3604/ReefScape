package frc.robot.subsystems.Manipulator;

import org.littletonrobotics.junction.AutoLog;

public interface ManipulatorIO {

  default void updateInputs(ManipulatorIOInputs inputs) {}

  @AutoLog
  class ManipulatorIOInputs {

    public boolean manipulatorLeftWheelMotorConnected = false;
    public boolean manipulatorRightWheelMotorConnected = false;

    public boolean manipulatorPieceSensor = false;
    public double manipulatorPieceSensorDistance;

    public double manipulatorLeftWheelMotorVoltage;
    public double manipulatorRightWheelMotorVoltage;

    public double manipulatorLeftWheelMotorCurrent;

    public double manipulatorLeftWheelStatorCurrent;
    public double manipulatorRightWheelMotorCurrent;

    public double manipulatorReefSensorDistance;

    public double manipulatorRightWheelTemperature;
    public double manipulatorLeftWheelTemperature;
  }

  default boolean getManipulatorSensor() {
    return false;
  }

  default void setWheelPower(double power) {}
}
