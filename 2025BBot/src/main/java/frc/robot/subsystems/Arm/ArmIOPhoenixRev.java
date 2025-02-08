package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.subsystems.Manipulator.ManipulatorConstants;

public class ArmIOPhoenixRev implements ArmIO {

    private final SparkFlex wrist;
    private final TalonFXS elbow;

    private final AbsoluteEncoder wristEncoder;

    private final PositionVoltage wristRequest;
    private final TorqueCurrentFOC torqueCurrentRequest;
    private final PositionTorqueCurrentFOC positionTorqueCurrentRequest;


    private final VoltageOut elbowRequest = new VoltageOut(0.0);

    public ArmIOPhoenixRev(){
        wrist = new SparkFlex(ArmConstants.wristID, MotorType.kBrushless);
        elbow = new TalonFXS(ArmConstants.elbowID);
        wristEncoder = wrist.getAbsoluteEncoder();

        wristRequest = new PositionVoltage(0).withSlot(0);
        torqueCurrentRequest = new TorqueCurrentFOC(0).withUpdateFreqHz(0);
        positionTorqueCurrentRequest = new PositionTorqueCurrentFOC(0).withUpdateFreqHz(0);

    // Phoenix6Util.applyAndCheckConfiguration(elbow, config);
    // Phoenix6Util.applyAndCheckConfiguration(wrist, config);

        var slot0Configs = new Slot0Configs();
        slot0Configs.kS = 0.1; 
        slot0Configs.kV = 0.12;
        slot0Configs.kP = ArmConstants.elbowP;
        slot0Configs.kI = ArmConstants.elbowI;
        slot0Configs.kD = ArmConstants.elbowD; 

        elbow.getConfigurator().apply(slot0Configs);

        SparkFlexConfig wristConfig = new SparkFlexConfig();
        wristConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        wristConfig.encoder
            .positionConversionFactor(1000)
            .velocityConversionFactor(1000);
        wristConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(ManipulatorConstants.openingMotorP, ManipulatorConstants.openingMotorI, ManipulatorConstants.openingMotorD);
    
        wrist.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    }

     @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.elbowMotorConnected = BaseStatusSignal.refreshAll(
                        elbow.getMotorVoltage(),
                        elbow.getSupplyCurrent(),
                        elbow.getDeviceTemp(),
                        elbow.getVelocity())
                .isOK();
        // inputs.wristMotorConnected = BaseStatusSignal.refreshAll(
        //                 wrist.getMotorVoltage(),
        //                 wrist.getSupplyCurrent(),
        //                 wrist.getDeviceTemp(),
        //                 wrist.getVelocity())
        //         .isOK();
        inputs.elbowMotorVoltage = elbow.getMotorVoltage().getValueAsDouble();
        inputs.elbowMotorCurrent = elbow.getSupplyCurrent().getValueAsDouble();
        inputs.wristMotorVoltage = wrist.getBusVoltage() * wrist.getAppliedOutput();
        inputs.wristMotorCurrent = wrist.getOutputCurrent();
        inputs.wristPosition = wristEncoder.getPosition();
    }

    @Override
    public void setWristMotorVoltage(double voltage) {
        wrist.setVoltage(voltage);
    }

    @Override
    public void setElbowMotorVoltage(double voltage) {
        elbow.setVoltage(voltage);
    }

    @Override
    public void setElbowPosition(double position){
        elbow.setControl(wristRequest.withPosition(position));
    }

    @Override
    public void setWristPosition(double position){
        wrist.getClosedLoopController().setReference(position, SparkFlex.ControlType.kPosition);
    }
}
