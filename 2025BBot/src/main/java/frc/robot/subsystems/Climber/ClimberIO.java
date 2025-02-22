package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  default void updateInputs(ClimberIOInputs inputs) {}

  @AutoLog
  class ClimberIOInputs {

    public boolean climberMotorConnected = false;
    public double climberMotorVoltage;
    public double climberMotorCurrent;

    public boolean climberEncoderConnected = false;
    public double climberEncoderPosition;
    public double climberVelocity;
  }

  default void setVoltage(double voltage) {}

  default void setPosition(double position) {}

  default void setPower(double power) {}

  default void setPID(double kP, double kI, double kD) {}
}
