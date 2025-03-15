package frc.robot.subsystems.Manipulator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.SparkUtil;

public class ManipulatorIOPhoenixRev implements ManipulatorIO {

  private TalonFXS leftWheel, rightWheel;
  private SparkFlex opening;
  private DigitalInput manipulatorSensor;
  private AbsoluteEncoder openingAbsoluteEncoder;
  private SparkFlexConfig openingConfig;
  // private LaserCan manipulatorDistanceSensor;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  // digital ID sensor is 3

  private final VoltageOut leftWheelRequest = new VoltageOut(0.0);
  private final VelocityVoltage wheelVelocityRequest = new VelocityVoltage(0).withSlot(0);
  private final VelocityTorqueCurrentFOC velocityTorqueControlRequest;
  private final TorqueCurrentFOC torqueControlRequest;

  public ManipulatorIOPhoenixRev() {

    // var slot0Configs = new Slot0Configs();
    // slot0Configs.kS = 0.1; // Add 0.1 V output to overcome static friction
    // slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    // slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
    // slot0Configs.kI = 0; // no output for integrated error
    // slot0Configs.kD = 0; // no output for error derivative

    // leftWheel.getConfigurator().apply(slot0Configs);

    leftWheel = new TalonFXS(ManipulatorConstants.leftWheelMotorID);
    rightWheel = new TalonFXS(ManipulatorConstants.rightWheelMotorID);
    // opening = new SparkFlex(ManipulatorConstants.openingMotorID, MotorType.kBrushless);
    manipulatorSensor = new DigitalInput(ManipulatorConstants.manipulatorSensorID);
    // manipulatorDistanceSensor = new LaserCan(51);
    velocityTorqueControlRequest = new VelocityTorqueCurrentFOC(0).withUpdateFreqHz(0);
    torqueControlRequest = new TorqueCurrentFOC(0).withUpdateFreqHz(0);
    // openingAbsoluteEncoder = opening.getAbsoluteEncoder();

    rightWheel.setControl(new Follower(leftWheel.getDeviceID(), true));

    // try {
    //   manipulatorDistanceSensor.setRangingMode(RangingMode.SHORT);
    //   manipulatorDistanceSensor.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 4, 4));
    // } catch (ConfigurationFailedException e) {
    //   // TODO Auto-generated catch block
    //   e.printStackTrace();
    // }

    TalonFXSConfiguration config = new TalonFXSConfiguration();
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 20;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 40;
    config.Voltage.PeakForwardVoltage = 12.0;
    config.Voltage.PeakReverseVoltage = -12.0;
    config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;

    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = 0.1; // Add 0.1 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kP = ManipulatorConstants.wheelP; // An error of 1 rps results in 0.11 V output
    slot0Configs.kI = ManipulatorConstants.wheelI; // no output for integrated error
    slot0Configs.kD = ManipulatorConstants.wheelD; // no output for error derivative

    leftWheel.getConfigurator().apply(slot0Configs);

    velocity = leftWheel.getVelocity();
    appliedVoltage = leftWheel.getMotorVoltage();
    supplyCurrent = leftWheel.getSupplyCurrent();
    torqueCurrent = leftWheel.getTorqueCurrent();

    PhoenixUtil.tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                50.0, velocity, appliedVoltage, supplyCurrent, torqueCurrent));

    PhoenixUtil.tryUntilOk(5, () -> leftWheel.getConfigurator().apply(config, 0.25));
    PhoenixUtil.tryUntilOk(5, () -> rightWheel.getConfigurator().apply(config, 0.25));

    // openingConfig = new SparkFlexConfig();
    // openingConfig.inverted(true).idleMode(IdleMode.kBrake);
    // openingConfig
    //     .closedLoop
    //     .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
    //     .pid(
    //         ManipulatorConstants.openingMotorP,
    //         ManipulatorConstants.openingMotorI,
    //         ManipulatorConstants.openingMotorD)
    //     .maxOutput(0.5)
    //     .minOutput(-0.5);
    // openingConfig.closedLoopRampRate(0.5);
    // SparkUtil.tryUntilOk(
    //     opening,
    //     5,
    //     () ->
    //         opening.configure(
    //             openingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ManipulatorIOInputs inputs) {
    inputs.manipulatorLeftWheelMotorConnected =
        BaseStatusSignal.refreshAll(
                leftWheel.getMotorVoltage(), leftWheel.getSupplyCurrent(), leftWheel.getVelocity())
            .isOK();
    inputs.manipulatorRightWheelMotorConnected =
        BaseStatusSignal.refreshAll(
                rightWheel.getMotorVoltage(),
                rightWheel.getSupplyCurrent(),
                rightWheel.getVelocity())
            .isOK();
    // inputs.manipulatorOpeningMotorConnected = BaseStatusSignal.refreshAll(
    // opening.getBusVoltage(),
    // opening.getOutputCurrent())
    // .isOK();
    inputs.manipulatorLeftWheelMotorVoltage = leftWheel.getMotorVoltage().getValueAsDouble();
    inputs.manipulatorLeftWheelMotorCurrent = leftWheel.getSupplyCurrent().getValueAsDouble();
    inputs.manipulatorLeftRPM = leftWheel.getVelocity().getValueAsDouble();
    inputs.manipulatorRightWheelMotorVoltage = rightWheel.getMotorVoltage().getValueAsDouble();
    inputs.manipulatorRightWheelMotorCurrent = rightWheel.getSupplyCurrent().getValueAsDouble();
    inputs.manipulatorRightRPM = rightWheel.getVelocity().getValueAsDouble();
    // inputs.manipulatorOpeningMotorVoltage = opening.getBusVoltage();
    // inputs.manipulatorOpeningMotorCurrent = opening.getOutputCurrent();
    // inputs.manipulatorOpeningMotorPos = openingAbsoluteEncoder.getPosition();
    // inputs.manipulatorOpeningMotorVelocity = openingAbsoluteEncoder.getVelocity();
    inputs.manipulatorSensor = !manipulatorSensor.get();

    // inputs.manipulatorDistance = manipulatorDistanceSensor.getMeasurement().distance_mm;
  }

  @Override
  public void setVoltage(double voltage) {
    leftWheel.setControl(leftWheelRequest.withOutput(MathUtil.clamp(voltage, -12, 12)));
  }

  @Override
  public void setOpeningVoltage(double voltage) {
    opening.setVoltage(voltage);
  }

  @Override
  public void setRPM(double RPM) {
    // leftWheel.setControl(
    // wheelVelocityRequest.withVelocity(RPM).withFeedForward(ManipulatorConstants.wheelFF));
    leftWheel.setControl(
        velocityTorqueControlRequest
            .withVelocity(RPM)
            .withFeedForward(ManipulatorConstants.wheelFF));
  }

  @Override
  public void setCurrent(double current) {
    leftWheel.setControl(torqueControlRequest.withOutput(current));
  }

  @Override
  public void setOpeningPos(double position) {
    opening.getClosedLoopController().setReference(position, SparkFlex.ControlType.kPosition);
  }

  @Override
  public boolean getManipulatorSensor() {
    return !manipulatorSensor.get();
  }

  public void setOpeningPower(double power) {
    opening.set(power);
  }

  public void setWheelPower(double power) {
    leftWheel.set(power);
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    openingConfig.closedLoop.pid(kP, kI, kD);
    SparkUtil.tryUntilOk(
        opening,
        5,
        () ->
            opening.configure(
                openingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }
}
