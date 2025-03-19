package frc.robot.subsystems.Manipulator;

import org.littletonrobotics.junction.AutoLog;

public interface ManipulatorIO {

  default void updateInputs(ManipulatorIOInputs inputs) {}

  @AutoLog
  class ManipulatorIOInputs {

    public boolean manipulatorLeftWheelMotorConnected = false;
    public boolean manipulatorRightWheelMotorConnected = false;
    public boolean manipulatorOpeningMotorConnected = false;

    public boolean manipulatorSensor = false;

    public double manipulatorLeftWheelMotorVoltage;
    public double manipulatorRightWheelMotorVoltage;
    public double manipulatorOpeningMotorVoltage;

    public double manipulatorLeftWheelMotorCurrent;
    public double manipulatorRightWheelMotorCurrent;
    public double manipulatorOpeningMotorCurrent;

    public double manipulatorOpeningMotorPos;

    public double manipulatorOpeningMotorVelocity;

    public double manipulatorRightRPM;
    public double manipulatorLeftRPM;

    public double manipulatorDistance;
    public boolean laserCANIsConnected = false;

    public double manipulatorRightWheelTemperature;
    public double manipulatorLeftWheelTemperature;
  }

  default void setOpeningPos(double position) {}

  default void setVoltage(double voltage) {}

  default void setRPM(double RPM) {}

  default void setOpeningVoltage(double voltage) {}

  default boolean getManipulatorSensor() {
    return false;
  }

  default void setWheelPower(double power) {}

  default void setOpeningPower(double power) {}

  default void setCurrent(double current) {}

  default void setPID(double kP, double kI, double kD) {}
}
