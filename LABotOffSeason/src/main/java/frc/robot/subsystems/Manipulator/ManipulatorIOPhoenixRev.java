package frc.robot.subsystems.Manipulator;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class ManipulatorIOPhoenixRev implements ManipulatorIO {

  private TalonFXS leftWheel, rightWheel;
  private LaserCan manipulatorGamePieceSensor;
  private LaserCan manipulatorBranchDetectionSensor;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;

  public ManipulatorIOPhoenixRev() {

    leftWheel = new TalonFXS(ManipulatorConstants.leftWheelMotorID);
    rightWheel = new TalonFXS(ManipulatorConstants.rightWheelMotorID);
    manipulatorGamePieceSensor = new LaserCan(ManipulatorConstants.manipulatorGamePieceSensorID);
    manipulatorBranchDetectionSensor =
        new LaserCan(ManipulatorConstants.manipulatorBranchDetectionSensorID);

    rightWheel.setControl(new Follower(leftWheel.getDeviceID(), true));

    try {
      manipulatorBranchDetectionSensor.setRangingMode(RangingMode.SHORT);
      manipulatorBranchDetectionSensor.setRegionOfInterest(
          new LaserCan.RegionOfInterest(8, 8, 6, 8));
      manipulatorBranchDetectionSensor.setTimingBudget(TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      e.printStackTrace();
    }

    try {
      manipulatorGamePieceSensor.setRangingMode(RangingMode.SHORT);
      manipulatorGamePieceSensor.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 6, 8));
      manipulatorGamePieceSensor.setTimingBudget(TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      e.printStackTrace();
    }

    TalonFXSConfiguration config = new TalonFXSConfiguration();
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.SupplyCurrentLimitEnable = false;
    config.CurrentLimits.SupplyCurrentLimit = 40; // was 20 before changing algae intake
    config.CurrentLimits.StatorCurrentLimitEnable = false;
    config.CurrentLimits.StatorCurrentLimit = 60;
    config.Voltage.PeakForwardVoltage = 12.0;
    config.Voltage.PeakReverseVoltage = -12.0;
    config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;

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

    inputs.manipulatorLeftWheelMotorVoltage = leftWheel.getMotorVoltage().getValueAsDouble();
    inputs.manipulatorLeftWheelMotorCurrent = leftWheel.getSupplyCurrent().getValueAsDouble();
    inputs.manipulatorLeftWheelStatorCurrent = leftWheel.getStatorCurrent().getValueAsDouble();
    inputs.manipulatorLeftRPM = leftWheel.getVelocity().getValueAsDouble();
    inputs.manipulatorRightWheelMotorVoltage = rightWheel.getMotorVoltage().getValueAsDouble();
    inputs.manipulatorRightWheelMotorCurrent = rightWheel.getSupplyCurrent().getValueAsDouble();
    inputs.manipulatorRightRPM = rightWheel.getVelocity().getValueAsDouble();

    inputs.manipulatorLeftWheelTemperature = leftWheel.getDeviceTemp().getValueAsDouble();
    inputs.manipulatorRightWheelTemperature = rightWheel.getDeviceTemp().getValueAsDouble();

    inputs.manipulatorGamePieceDetectionCurrentDistance =
        manipulatorGamePieceSensor.getMeasurement().status
                == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT
            ? manipulatorGamePieceSensor.getMeasurement().distance_mm
            : 3604;

    inputs.manipulatorGamePieceDetected =
        inputs.manipulatorGamePieceDetectionCurrentDistance
            <= ManipulatorConstants.hasGamePieceThreshold;

    inputs.manipulatorBranchDetectionCurrentDistance =
        manipulatorBranchDetectionSensor.getMeasurement().status
                == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT
            ? manipulatorBranchDetectionSensor.getMeasurement().distance_mm
            : 3604;
  }

  @Override
  public void setWheelPower(double power) {
    leftWheel.set(power);
  }
}
