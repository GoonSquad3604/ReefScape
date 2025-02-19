package frc.robot.subsystems.Elevator;

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
  // private DigitalInput limitSwitchLeft;
  // private DigitalInput limitSwitchRight;

  private SparkFlexConfig config;

  public ElevatorIONeo() {

    // initializes motors
    leftMotor = new SparkFlex(ElevatorConstants.leftMotorID, MotorType.kBrushless);
    rightMotor = new SparkFlex(ElevatorConstants.rightMotorID, MotorType.kBrushless);

    // left motor config
    config = new SparkFlexConfig();

    config.inverted(false).idleMode(IdleMode.kBrake);
    // config.encoder.positionConversionFactor(1000).velocityConversionFactor(1000);
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ElevatorConstants.elevatorP, ElevatorConstants.elevatorI, ElevatorConstants.elevatorD);

    // right motor config
    SparkFlexConfig rightConfig = new SparkFlexConfig();

    rightConfig.idleMode(IdleMode.kBrake).follow(ElevatorConstants.leftMotorID, true);
    rightConfig.encoder.positionConversionFactor(1000).velocityConversionFactor(1000);
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

    // inputs.limitSwitchLeft = leftMotor.getForwardLimitSwitch().isPressed();
    // inputs.limitSwitchRight = limitSwitchRight.get();

    double leftVoltage = leftMotor.getAppliedOutput() * leftMotor.getBusVoltage();
    double rightVoltage = rightMotor.getAppliedOutput() * rightMotor.getBusVoltage();

    inputs.MotorLeftVoltage = leftVoltage;
    inputs.MotorRightVoltage = rightVoltage;

    inputs.MotorLeftCurrent = leftMotor.getOutputCurrent();
    inputs.MotorRightCurrent = leftMotor.getOutputCurrent();

    inputs.MotorLeftPos = leftMotor.getAbsoluteEncoder().getPosition();
    inputs.MotorRightPos = rightMotor.getAbsoluteEncoder().getPosition();
    inputs.HeightInInches =
        ElevatorConstants.homeOffset
            + leftMotor.getEncoder().getPosition()
                * ElevatorConstants.gearRatio
                * ElevatorConstants.pulleyCircumference;
  }

  @Override
  public void setPos(double position) {

    // set l & r motor to a set position
    leftMotor.getClosedLoopController().setReference(position, ControlType.kPosition);
  }

  @Override
  public void setPosInches(double position) {

    // calculates the position the motors should go to, then goes to position
    leftMotor
        .getClosedLoopController()
        .setReference(findPosInInches(position), ControlType.kPosition);
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
  public double findPosInInches(double pos) {

    // gets encoder pos from inches
    return ElevatorConstants.homeOffset
        + pos * ElevatorConstants.gearRatio * ElevatorConstants.pulleyCircumference;
  }

  @Override
  public double findPosFromInches(double posInInches) {

    // gets the pos from a given pos in inches
    return posInInches / ElevatorConstants.gearRatio / ElevatorConstants.pulleyCircumference
        - ElevatorConstants.homeOffset;
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
}
