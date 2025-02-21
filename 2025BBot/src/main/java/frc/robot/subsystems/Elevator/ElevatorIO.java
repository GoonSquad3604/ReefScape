package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  default void updateInputs(ElevatorIOInputs inputs) {}

  @AutoLog
  class ElevatorIOInputs {

    // silly little code -lucas

    public boolean MotorLeftConnected = false;
    public boolean MotorRightConnected = false;
    public boolean limitSwitchLeft = false;
    public boolean limitSwitchRight = false;

    public double MotorLeftVoltage;
    public double MotorRightVoltage;

    public double MotorLeftCurrent;
    public double MotorRightCurrent;

    public double MotorLeftPos;
    public double MotorRightPos;
    public double HeightInInches;
  }

  default boolean checkLimitSwitch() {

    return false;
  }

  default void setPos(double position) {}

  default void setPosInches(double position) {}

  default void setVoltage(double voltage) {}

  default void setToZero() {}

  default double findPosInInches(double pos) {

    return pos;
  }

  default double getPos() {

    return 0;
  }

  default double findPosFromInches(double posInInches) {

    return posInInches;
  }

  default void setPower(double power) {}

  default void setPID(double kP, double kI, double kD) {}

  default void setPositionClosedLoopWithFF(double position, double arbFF) {}
}
// mandatory end comment
