package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  default void updateInputs(ElevatorIOInputs inputs) {}

  @AutoLog
  class ElevatorIOInputs {
    public boolean limitSwitchLeft = false;

    public double MotorLeftVoltage;
    public double MotorRightVoltage;
    public double MotorLeftVelocity;

    public double MotorLeftCurrent;
    public double MotorRightCurrent;

    public double MotorLeftPos;
    public double MotorRightPos;
    public double HeightInInches;
  }

  default boolean checkLimitSwitch() {
    return false;
  }

  default void setToZero() {}

  default double findPosInInches(double pos) {
    return pos;
  }

  default double getPos() {
    return 0;
  }

  default void setPower(double power) {}

  default void setPositionClosedLoopWithFF(double position, double arbFF) {}
}
