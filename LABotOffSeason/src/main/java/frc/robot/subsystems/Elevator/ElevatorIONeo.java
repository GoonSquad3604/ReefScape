package frc.robot.subsystems.Elevator;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
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

    // left motor config
    config = new SparkFlexConfig();

    config.inverted(true).idleMode(IdleMode.kBrake);
    config.encoder.positionConversionFactor(1).velocityConversionFactor(1);

    // right motor config
    SparkFlexConfig rightConfig = new SparkFlexConfig();

    rightConfig.idleMode(IdleMode.kBrake).follow(ElevatorConstants.leftMotorID, true);

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
  }

  // updates IO
  @Override
  public void updateInputs(ElevatorIOInputs inputs) {

    inputs.limitSwitchLeft = leftMotor.getReverseLimitSwitch().isPressed();

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
  public void setToZero() {
    // sets the motor encoder to zero
    leftMotor.getEncoder().setPosition(0);
  }

  @Override
  public void setPower(double power) {
    // sets the speed to a given power
    leftMotor.set(power);
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
