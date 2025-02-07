package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Encoder;

public class ArmIOPhoenix implements ArmIO {

    private SparkFlex wrist;
    private TalonFX elbow;

    private AbsoluteEncoder wristEncoder;


    private final VoltageOut elbowRequest = new VoltageOut(0.0);
    private final VoltageOut wristRequest = new VoltageOut(0.0);

    public ArmIOPhoenix(){
        wrist = new SparkFlex(ArmConstants.wristID, MotorType.kBrushless);
        elbow = new TalonFX(ArmConstants.elbowID);

        wristEncoder = wrist.getAbsoluteEncoder();

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 80;
        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;

        // Phoenix6Util.applyAndCheckConfiguration(elbow, config);
        // Phoenix6Util.applyAndCheckConfiguration(wrist, config);
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
        inputs.wristMotorVoltage = wrist.getBusVoltage();
        inputs.wristMotorCurrent = wrist.getOutputCurrent();
    }

     @Override
    public void setElbowMotorVoltage(double voltage) {
        elbow.setControl(elbowRequest.withOutput(MathUtil.clamp(voltage, -12.0, 12.0)));
    }

    @Override
    public void setWristMotorVoltage(double voltage) {
        //wrist.setControl(wristRequest.withOutput(MathUtil.clamp(voltage, -12.0, 12.0)));
    }

    @Override
    public void setElbowPosition(double position){
        
    }

    @Override
    public void setWristPosition(double position){
        
    }
}