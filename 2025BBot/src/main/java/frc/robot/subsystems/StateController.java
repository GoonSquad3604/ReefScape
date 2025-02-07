package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.RobotMode;

public class StateController extends SubsystemBase{
    public static StateController _instance;

    @AutoLogOutput 
    private RobotMode m_Mode;

    public StateController(){
        setCoral();
    }

    public static StateController getInstance() {

        if (_instance == null) {
          _instance = new StateController();
        }
        return _instance;
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
}
