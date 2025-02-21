package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.util.PhoenixUtil;

public class ClimberIOPhoenix implements ClimberIO {

  private TalonFX climberMotor;
  // private CANcoder climberEncoder;

  // position closed-loop request
  private final PositionVoltage climberPositionrequest;
  private final PositionTorqueCurrentFOC positionTorqueCurrentRequest;

  private Slot0Configs slot0Configs;
  private TalonFXConfiguration config;
  // private CANcoderConfiguration encoderConfig;

  public ClimberIOPhoenix() {

    // declare the motor, encoder, and configs
    climberMotor = new TalonFX(ClimberConstants.motorID);
    // climberEncoder = new CANcoder(ClimberConstants.encoderID);

    config = new TalonFXConfiguration();
    // encoderConfig = new CANcoderConfiguration();

    // create position request
    climberPositionrequest = new PositionVoltage(0).withSlot(0);
    positionTorqueCurrentRequest = new PositionTorqueCurrentFOC(0).withUpdateFreqHz(0);

    // motor configs
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 80;
    config.Voltage.PeakForwardVoltage = 12.0;
    config.Voltage.PeakReverseVoltage = -12.0;
    config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;

    config.Feedback.FeedbackRemoteSensorID = ClimberConstants.encoderID;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

    // encoder configs
    // CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    // config.Feedback.FeedbackRemoteSensorID = ClimberConstants.encoderID;
    // config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    // config.Feedback.withRemoteCANcoder(climberEncoder);

    // apply configs
    PhoenixUtil.tryUntilOk(5, () -> climberMotor.getConfigurator().apply(config));

    // configure and apply PID
    slot0Configs = new Slot0Configs();
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

    // inputs.climberEncoderConnected = climberEncoder.isConnected();
    // inputs.climberEncoderPosition = climberEncoder.getPosition();
    // inputs.climberVelocity = wristEncoder.getVelocity();

  }

  @Override
  public void setVoltage(double voltage) {
    climberMotor.setVoltage(voltage);
  }

  @Override
  public void setPosition(double position) {
    climberMotor.setControl(positionTorqueCurrentRequest.withPosition(position));
  }

  public void setPower(double power) {
    climberMotor.set(power);
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    slot0Configs.kP = kP;
    slot0Configs.kI = kI;
    slot0Configs.kD = kD;
    PhoenixUtil.tryUntilOk(5, () -> climberMotor.getConfigurator().apply(config));
  }
}
