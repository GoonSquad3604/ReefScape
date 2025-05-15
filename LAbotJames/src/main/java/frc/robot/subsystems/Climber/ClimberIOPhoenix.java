package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.util.PhoenixUtil;

public class ClimberIOPhoenix implements ClimberIO {

  private TalonFX climberMotor;
  private TalonFXConfiguration config;

  public ClimberIOPhoenix() {

    // declare motor and config
    climberMotor = new TalonFX(ClimberConstants.motorID);
    config = new TalonFXConfiguration();

    // motor config setup
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 80;
    config.Voltage.PeakForwardVoltage = 12.0;
    config.Voltage.PeakReverseVoltage = -12.0;
    config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;
    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = .5;

    // apply configs
    PhoenixUtil.tryUntilOk(5, () -> climberMotor.getConfigurator().apply(config));
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.climberMotorConnected =
        BaseStatusSignal.refreshAll(
                climberMotor.getMotorVoltage(),
                climberMotor.getSupplyCurrent(),
                climberMotor.getDeviceTemp(),
                climberMotor.getVelocity())
            .isOK();
    inputs.climberMotorVoltage = climberMotor.getMotorVoltage().getValueAsDouble();
    inputs.climberMotorCurrent = climberMotor.getSupplyCurrent().getValueAsDouble();
  }

  public void setPower(double power) {
    climberMotor.set(power);
  }
}
