package frc.robot.subsystems.Elevator;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import frc.robot.subsystems.Elevator.ElevatorConstants;

public class ElevatorIONeo implements ElevatorIO {

    //declares motors
    private SparkMax leftMotor;
    private SparkMax rightMotor;
    private DigitalInput limitSwitchLeft;
    private DigitalInput limitSwitchRight;
    

    public ElevatorIONeo(){

        //initializes motors
        leftMotor = new SparkMax(ElevatorConstants.leftMotorID, MotorType.kBrushless);
        rightMotor = new SparkMax(ElevatorConstants.rightMotorID, MotorType.kBrushless);

        limitSwitchLeft = new DigitalInput(ElevatorConstants.limitSwitch1ID);
        limitSwitchRight = new DigitalInput(ElevatorConstants.limitSwitch2ID);

    }

    //updates IO
    @Override
    public void updateInputs(ElevatorIOInputs inputs){

        inputs.limitSwitchLeft = limitSwitchLeft.get();
        inputs.limitSwitchRight = limitSwitchRight.get();

        // inputs.MotorLeftVoltage = leftMotor.getVoltage();
        // inputs.MotorRightVoltage = rightMotor.getVoltage();

        inputs.MotorLeftCurrent = leftMotor.getOutputCurrent();
        inputs.MotorRightCurrent = leftMotor.getOutputCurrent();

        inputs.MotorLeftPos = leftMotor.getAbsoluteEncoder().getPosition();
        inputs.MotorRightPos = rightMotor.getAbsoluteEncoder().getPosition();
        // inputs.HeightInInches = ;

    }

    @Override
    public void setPos(double position) {}

    @Override
    public void setPosInches(double position) {}

    @Override
    public void setVoltage(double voltage) {}

    @Override
    public void setToZero() {}

}
