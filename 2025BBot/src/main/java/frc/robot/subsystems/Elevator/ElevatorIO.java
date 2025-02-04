package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  default void updateInputs(ElevatorIOInputs inputs) {}

  @AutoLog
  class ElevatorIOInputs {

    // silly little code -lucas

    public boolean elevatorMotorLeftConnected = false;
    public boolean elevatorMotorRightConnected = false;
    public boolean limitSwitch = false;

    public double elevatorMotorLeftVoltage;
    public double elevatorMotorRightVoltage;

    public double elevatorMotorLeftCurrent;
    public double elevatorMotorRightCurrent;

    public double elevatorMotorLeftPos;
    public double elevatorMotorRightPos;
    public double elevatorHeightInInches;
  }

  default void setPos(double position) {}

  default void setPosInches(double position) {}

  default void setVoltage(double voltage) {}

  default void setToZero() {}
}
