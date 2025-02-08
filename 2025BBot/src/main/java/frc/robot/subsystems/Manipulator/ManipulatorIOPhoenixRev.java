package frc.robot.subsystems.Manipulator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;

public class ManipulatorIOPhoenixRev implements ManipulatorIO {

    private TalonFXS leftWheel, rightWheel;
    private SparkFlex opening;
    private DigitalInput manipulatorSensor;
    private AbsoluteEncoder openingAbsoluteEncoder;
    //digital ID sensor is 3
    
    
        
    
    private final VoltageOut leftWheelRequest = new VoltageOut(0.0);
    private final VoltageOut rightWheelRequest = new VoltageOut(0.0);
    private final VoltageOut openingRequest = new VoltageOut(0.0);
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
    
    public ManipulatorIOPhoenixRev(){
    
        var slot0Configs = new Slot0Configs();
        slot0Configs.kS = 0.1; // Add 0.1 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // no output for error derivative

        leftWheel.getConfigurator().apply(slot0Configs);

        leftWheel = new TalonFXS(ManipulatorConstants.leftWheelMotorID);
        rightWheel = new TalonFXS(ManipulatorConstants.rightWheelMotorID);
        opening = new SparkFlex(ManipulatorConstants.openingMotorID, MotorType.kBrushless);
        manipulatorSensor = new DigitalInput(ManipulatorConstants.manipulatorSensorID);
        openingAbsoluteEncoder = opening.getAbsoluteEncoder();

    


        rightWheel.setControl(new Follower(leftWheel.getDeviceID(), true));

        TalonFXSConfiguration config = new TalonFXSConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 80;
        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;
        SparkFlexConfig openingConfig = new SparkFlexConfig();
        openingConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        openingConfig.encoder
            .positionConversionFactor(1000)
            .velocityConversionFactor(1000);
        openingConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(ManipulatorConstants.openingMotorP, ManipulatorConstants.openingMotorI, ManipulatorConstants.openingMotorD);
    
    opening.configure(openingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

     @Override
    public void updateInputs(ManipulatorIOInputs inputs) {
inputs.manipulatorLeftWheelMotorConnected = BaseStatusSignal.refreshAll(
                        leftWheel.getMotorVoltage(),
                        leftWheel.getSupplyCurrent(),
                        leftWheel.getDeviceTemp(),
                        leftWheel.getVelocity())
                .isOK();
        inputs.manipulatorRightWheelMotorConnected = BaseStatusSignal.refreshAll(
                        rightWheel.getMotorVoltage(),
                        rightWheel.getSupplyCurrent(),
                        rightWheel.getDeviceTemp(),
                        rightWheel.getVelocity())
                .isOK();

        inputs.manipulatorOpeningMotorCurrent = opening.getOutputCurrent();
        inputs.manipulatorOpeningMotorVoltage = opening.getBusVoltage();
        inputs.manipulatorOpeningMotorPos = openingAbsoluteEncoder.getPosition();
        inputs.manipulatorLeftWheelMotorVoltage = leftWheel.getMotorVoltage().getValueAsDouble();
        inputs.manipulatorLeftWheelMotorCurrent = leftWheel.getSupplyCurrent().getValueAsDouble();
        inputs.manipulatorRightWheelMotorVoltage = rightWheel.getMotorVoltage().getValueAsDouble();
        inputs.manipulatorRightWheelMotorCurrent = rightWheel.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void setVoltage(double voltage){
        leftWheel.setControl(leftWheelRequest.withOutput(MathUtil.clamp(voltage, -12, 12)));
    }
    @Override
    public void setRPM(double RPM){
        leftWheel.setControl(m_request.withVelocity(RPM).withFeedForward(ManipulatorConstants.wheelFF));
    }
    @Override
    public void setOpeningPos(double position){
        opening.getClosedLoopController().setReference(position, SparkFlex.ControlType.kPosition);
    }
}
