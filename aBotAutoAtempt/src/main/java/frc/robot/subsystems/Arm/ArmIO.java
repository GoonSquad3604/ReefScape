// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

  default void updateInputs(ArmIOInputs inputs) {}

  @AutoLog
  class ArmIOInputs {

    public boolean elbowMotorConnected = false;
    public double elbowMotorVoltage;
    public double elbowMotorCurrent;
    public double elbowVelocity;
    public double wristVelocity;
    public double wristMotorVoltage;
    public double wristMotorCurrent;
    public double elbowPosition;
    public double wristPosition;
    public boolean elbowEncoderConnected = false;
    public boolean wristEncoderConnected = false;
  }

  default void setElbowPosition(double position) {}

  default void setWristPosition(double position) {}

  default void setElbowPower(double power) {}

  default void setWristPower(double power) {}

  default double getElbowPosition() {
    return 0;
  }

  default double getWristPosition() {
    return 0;
  }
}
