package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.util.PhoenixUtil;

public class ClimberIOPhoenix implements ClimberIO {

  private TalonFX climberMotor;
  // private CANcoder climberEncoder;

  // create a position closed-loop request, voltage output, slot 0 configs
  private final PositionVoltage climberPositionrequest;
  private final PositionTorqueCurrentFOC positionTorqueCurrentRequest;

  public ClimberIOPhoenix() {

    // declare the motor and encoder
    climberMotor = new TalonFX(ClimberConstants.motorID);
    // climberEncoder = new CANcoder(ClimberConstants.encoderID);

    climberPositionrequest = new PositionVoltage(0).withSlot(0);
    positionTorqueCurrentRequest = new PositionTorqueCurrentFOC(0).withUpdateFreqHz(0);

    // motor configs
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 80;
    config.Voltage.PeakForwardVoltage = 12.0;
    config.Voltage.PeakReverseVoltage = -12.0;
    config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;

    // encoder configs
    // CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    // config.Feedback.FeedbackRemoteSensorID = ClimberConstants.encoderID;
    // config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    // config.Feedback.withRemoteCANcoder(climberEncoder);

    // apply configs
    PhoenixUtil.tryUntilOk(5, () -> climberMotor.getConfigurator().apply(config));

    // configure PID, apply slot 0 gains
    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = ClimberConstants.p;
    slot0Configs.kI = ClimberConstants.i;
    slot0Configs.kD = ClimberConstants.d;

    climberMotor.getConfigurator().apply(slot0Configs);
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

  @Override
  public void setVoltage(double voltage) {
    climberMotor.setVoltage(voltage);
  }

  @Override
  public void setPosition(double position) {
    // climberMotor.setControl(climberPositionrequest.withPosition(position));
    climberMotor.setControl(positionTorqueCurrentRequest.withPosition(position));
  }

  public void setPower(double power) {
    climberMotor.set(power);
  }
}
