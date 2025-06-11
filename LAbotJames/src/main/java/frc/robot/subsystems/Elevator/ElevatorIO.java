package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  default void updateInputs(ElevatorIOInputs inputs) {}

  @AutoLog
  class ElevatorIOInputs {

    public boolean limitSwitchLeft = false;
    public boolean limitSwitchRight = false;

    public double MotorLeftVoltage;
    public double MotorRightVoltage;
    public double MotorLeftVelocity;

    public double MotorLeftCurrent;
    public double MotorRightCurrent;

    public double MotorLeftPos;
    public double MotorRightPos;
  }

  default boolean checkLimitSwitch() {

    return false;
  }

  default void setPos(double position) {}

  default void setVoltage(double voltage) {}

  default void setToZero() {}

  default double getPos() {

    return 0;
  }

  default void setPower(double power) {}

  default void setPID(double kP, double kI, double kD) {}

  default void setPositionClosedLoopWithFF(double position, double arbFF) {}
}