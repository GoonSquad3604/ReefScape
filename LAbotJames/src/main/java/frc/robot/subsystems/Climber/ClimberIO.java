package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  default void updateInputs(ClimberIOInputs inputs) {}

  @AutoLog
  class ClimberIOInputs {
    public boolean climberMotorConnected = false;
    public double climberMotorVoltage;
    public double climberMotorCurrent;
  }

  default void setPower(double power) {}
}
