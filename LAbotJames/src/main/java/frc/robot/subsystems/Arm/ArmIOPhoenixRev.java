package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class ArmIOPhoenixRev implements ArmIO {

  private final SparkFlex wrist;
  private final TalonFX elbow;

  private final AbsoluteEncoder wristEncoder;
  private final CANcoder elbowEncoder;

  private final PositionVoltage elbowRequest;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> tempCelsius;

  private TalonFXConfiguration elbowConfig;

  public ArmIOPhoenixRev() {
    wrist = new SparkFlex(ArmConstants.wristID, MotorType.kBrushless);
    elbow = new TalonFX(ArmConstants.elbowID);
    wristEncoder = wrist.getAbsoluteEncoder();
    elbowRequest = new PositionVoltage(0);

    elbowEncoder = new CANcoder(ArmConstants.elbowEncoderID);

    CANcoderConfiguration CANfig = new CANcoderConfiguration();

    CANfig.MagnetSensor.MagnetOffset = 0.449;
    elbowEncoder.getConfigurator().apply(CANfig);

    SparkFlexConfig wristConfig = new SparkFlexConfig();
    wristConfig.inverted(true).idleMode(IdleMode.kBrake);
    wristConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(ArmConstants.wristP, ArmConstants.wristI, ArmConstants.wristD);

    wrist.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    elbowConfig = new TalonFXConfiguration();

    elbowConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    elbowConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    elbowConfig.CurrentLimits.SupplyCurrentLimit = 40;
    elbowConfig.Feedback.FeedbackRemoteSensorID = ArmConstants.elbowEncoderID;
    elbowConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    elbowConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    elbowConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.5;
    elbowConfig.Slot0 =
        new Slot0Configs()
            .withKP(ArmConstants.elbowP)
            .withKI(ArmConstants.elbowI)
            .withKD(ArmConstants.elbowD);
    elbowConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;
    PhoenixUtil.tryUntilOk(5, () -> elbow.getConfigurator().apply(elbowConfig));

    position = elbow.getPosition();
    velocity = elbow.getVelocity();
    appliedVoltage = elbow.getMotorVoltage();
    supplyCurrent = elbow.getSupplyCurrent();
    torqueCurrent = elbow.getTorqueCurrent();
    tempCelsius = elbow.getDeviceTemp();

    PhoenixUtil.tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                position,
                velocity,
                appliedVoltage,
                supplyCurrent,
                torqueCurrent,
                tempCelsius));
    PhoenixUtil.tryUntilOk(5, () -> elbow.optimizeBusUtilization(0, 1.0));

    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = 0.31;
    slot0Configs.kV = 0.12;
    slot0Configs.kP = ArmConstants.elbowP;
    slot0Configs.kI = ArmConstants.elbowI;
    slot0Configs.kD = ArmConstants.elbowD;

    elbow.getConfigurator().apply(slot0Configs);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.elbowMotorConnected =
        BaseStatusSignal.refreshAll(
                elbow.getMotorVoltage(),
                elbow.getSupplyCurrent(),
                elbow.getDeviceTemp(),
                elbow.getVelocity())
            .isOK();

    inputs.wristEncoderConnected = wrist.getBusVoltage() < 5000 && wrist.getBusVoltage() > -5000;
    inputs.elbowMotorVoltage = elbow.getMotorVoltage().getValueAsDouble();
    inputs.elbowMotorCurrent = elbow.getSupplyCurrent().getValueAsDouble();
    inputs.elbowVelocity = elbowEncoder.getVelocity().getValueAsDouble();
    inputs.elbowEncoderConnected = elbowEncoder.isConnected();
    inputs.elbowPosition = elbowEncoder.getAbsolutePosition().getValueAsDouble();
    inputs.wristVelocity = wristEncoder.getVelocity();
    inputs.wristMotorVoltage = wrist.getBusVoltage() * wrist.getAppliedOutput();
    inputs.wristMotorCurrent = wrist.getOutputCurrent();

    inputs.wristPosition = wristEncoder.getPosition();
  }

  @Override
  public void setElbowPosition(double position) {
    elbow.setControl(elbowRequest.withPosition(position));
  }

  @Override
  public void setWristPosition(double position) {
    wrist
        .getClosedLoopController()
        .setReference(
            position, SparkFlex.ControlType.kPosition, ClosedLoopSlot.kSlot0, ArmConstants.wristFF);
  }

  @Override
  public void setElbowPower(double power) {
    elbow.set(power);
  }

  @Override
  public void setWristPower(double power) {
    wrist.set(power);
  }

  @Override
  public double getWristPosition() {
    return wristEncoder.getPosition();
  }

  @Override
  public double getElbowPosition() {
    return elbowEncoder.getPosition().getValueAsDouble();
  }
}
