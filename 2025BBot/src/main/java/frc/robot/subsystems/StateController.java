package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Manipulator.Manipulator;
import frc.robot.util.RobotMode;

public class StateController extends SubsystemBase{
    public static StateController _instance;

    @AutoLogOutput 
    private RobotMode m_Mode;

    public StateController(){
        setCoral();
    }

    public void setCoral(){
        m_Mode = RobotMode.CORAL;
    }
    public void setAlgae(){
        m_Mode = RobotMode.ALGAE;
    }
    public boolean isCoralMode(){
        return m_Mode == RobotMode.CORAL;
    }
    public boolean isAlgaeMode(){
        return m_Mode == RobotMode.ALGAE;
    }
    public RobotMode getMode(){
        return m_Mode;
    }
    public Command setCoralMode(Manipulator manipulator){
        return run(() -> {
            setCoral();
            manipulator.setOpeningToCoral();
        });
    }
    public Command setAlgaeMode(Manipulator manipulator){
        return runEnd(() -> setAlgae(), () -> manipulator.setOpeningToAlgae());
    }
    
}
