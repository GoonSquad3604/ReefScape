package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimberIOPhoenix implements ClimberIO {

    private TalonFX climberMotor;

    // create a position closed-loop request, voltage output, slot 0 configs
    final PositionVoltage climberPositionrequest = new PositionVoltage(0).withSlot(0);

    public ClimberIOPhoenix() {
        
        //declare motor and encoder
        climberMotor = new TalonFX(ClimberConstants.ID);


        //Configure the motor
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 80;
        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;

        
        //configure PID, set slot 0 gains
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = ClimberConstants.p;
        slot0Configs.kI = ClimberConstants.i;
        slot0Configs.kD = ClimberConstants.d;

        climberMotor.getConfigurator().apply(slot0Configs);

    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.climberMotorConnected = BaseStatusSignal.refreshAll(
            climberMotor.getMotorVoltage(),
            climberMotor.getSupplyCurrent(),
            climberMotor.getDeviceTemp(),
            climberMotor.getVelocity())
                .isOK();
        
        inputs.climberMotorVoltage = climberMotor.getMotorVoltage().getValueAsDouble();
        inputs.climberMotorCurrent = climberMotor.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void setVoltage(double voltage){
        climberMotor.setVoltage(voltage);
    }

    @Override
    public void setPosition(double position){
        //climberMotor.setPosition(position);
        climberMotor.setControl(climberPositionrequest.withPosition(position));
    }

}