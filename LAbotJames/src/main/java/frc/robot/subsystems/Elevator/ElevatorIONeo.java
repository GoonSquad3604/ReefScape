package frc.robot.subsystems.Elevator;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.util.SparkUtil;

public class ElevatorIONeo implements ElevatorIO {

  // declares motors
  private SparkFlex leftMotor;
  private SparkFlex rightMotor;
  // private LaserCan laserCan;
  // private DigitalInput limitSwitchLeft;
  // private DigitalInput limitSwitchRight;

  private SparkFlexConfig config;

  public ElevatorIONeo() {

    // initializes motors
    leftMotor = new SparkFlex(ElevatorConstants.leftMotorID, MotorType.kBrushless);
    rightMotor = new SparkFlex(ElevatorConstants.rightMotorID, MotorType.kBrushless);

    // initialize the laser cannon
    // laserCan = new LaserCan(ElevatorConstants.laserCanID);
    // try {
    //   laserCan.setRangingMode(RangingMode.SHORT);
    //   laserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16 - 0, 16));
    // } catch (ConfigurationFailedException e) {
    //   e.printStackTrace();
    // }

    // left motor config
    config = new SparkFlexConfig();

    config.inverted(true).idleMode(IdleMode.kBrake);
    config.encoder.positionConversionFactor(1).velocityConversionFactor(1);
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ElevatorConstants.elevatorP, ElevatorConstants.elevatorI, ElevatorConstants.elevatorD);
    // config.closedLoopRampRate(1 - 0 + 0);

    // right motor config
    SparkFlexConfig rightConfig = new SparkFlexConfig();

    rightConfig.idleMode(IdleMode.kBrake).follow(ElevatorConstants.leftMotorID, true);
    // rightConfig.encoder.positionConversionFactor(1000).velocityConversionFactor(1000);
    rightConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ElevatorConstants.elevatorP, ElevatorConstants.elevatorI, ElevatorConstants.elevatorD);

    // sets configs in motion
    SparkUtil.tryUntilOk(
        leftMotor,
        5,
        () ->
            leftMotor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    SparkUtil.tryUntilOk(
        rightMotor,
        5,
        () ->
            rightMotor.configure(
                rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // initialializes limit switches
    // limitSwitchLeft = new DigitalInput(ElevatorConstants.limitSwitch1ID);
    // limitSwitchRight = new DigitalInput(ElevatorConstants.limitSwitch2ID);
  }

  // updates IO
  @Override
  public void updateInputs(ElevatorIOInputs inputs) {

    inputs.limitSwitchLeft = leftMotor.getReverseLimitSwitch().isPressed();
    // inputs.limitSwitchRight = limitSwitchRight.get();

    double leftVoltage = leftMotor.getAppliedOutput() * leftMotor.getBusVoltage();
    double rightVoltage = rightMotor.getAppliedOutput() * rightMotor.getBusVoltage();

    inputs.MotorLeftVoltage = leftVoltage;
    inputs.MotorRightVoltage = rightVoltage;

    inputs.MotorLeftCurrent = leftMotor.getOutputCurrent();
    inputs.MotorRightCurrent = rightMotor.getOutputCurrent();
    inputs.MotorLeftVelocity = leftMotor.getEncoder().getVelocity();

    inputs.MotorLeftPos = leftMotor.getEncoder().getPosition();
    inputs.MotorRightPos = rightMotor.getEncoder().getPosition();
  }

  @Override
  public void setPos(double position) {

    // set l & r motor to a set position
    leftMotor.getClosedLoopController().setReference(position, ControlType.kPosition);
  }

  @Override
  public void setVoltage(double voltage) {

    // set l & r motor to a given voltage
    leftMotor.getClosedLoopController().setReference(voltage, ControlType.kVoltage);
  }

  @Override
  public void setToZero() {

    // sets the motor to zero
    leftMotor.getEncoder().setPosition(0);
  }

  @Override
  public void setPower(double power) {

    // sets the speed to a given power
    leftMotor.set(power);
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    config.closedLoop.pid(kP, kI, kD);
    SparkUtil.tryUntilOk(
        leftMotor,
        5,
        () ->
            leftMotor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void setPositionClosedLoopWithFF(double position, double arbFF) {

    leftMotor
        .getClosedLoopController()
        .setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0, arbFF);
  }

  @Override
  public boolean checkLimitSwitch() {

    return leftMotor.getReverseLimitSwitch().isPressed();
  }

  @Override
  public double getPos() {

    return leftMotor.getEncoder().getPosition();
  }
}