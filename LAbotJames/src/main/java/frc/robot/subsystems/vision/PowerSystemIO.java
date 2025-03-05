package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

public interface PowerSystemIO {

  default void updateInputs(PowerSystemIOInputs inputs) {}

  @AutoLog
  class PowerSystemIOInputs {
    public boolean mitoCANdriaConnected = false;
    public boolean mitoFiveVoltAEnabled = false;

    public double mitoUSBOneCurrent;
    public double mitoUSBTwoCurrent;
    public double mitoFiveVoltACurrent;
  }

  default void setFiveVoltAEnabled(boolean enabled) {}
}
