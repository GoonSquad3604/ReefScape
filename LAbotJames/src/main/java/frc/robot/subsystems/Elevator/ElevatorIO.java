package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  default void updateInputs(ElevatorIOInputs inputs) {}

  @AutoLog
  class ElevatorIOInputs {

    public boolean limitSwitch = false;

    public double motorLeftVoltage;
    public double motorRightVoltage;
    public double motorLeftVelocity;

    public double motorLeftCurrent;
    public double motorRightCurrent;

    public double motorLeftPos;
    public double motorRightPos;
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
