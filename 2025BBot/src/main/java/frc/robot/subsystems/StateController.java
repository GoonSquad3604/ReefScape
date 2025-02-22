package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Manipulator.Manipulator;
import frc.robot.util.LevelState;
import frc.robot.util.RobotMode;
import org.littletonrobotics.junction.AutoLogOutput;

public class StateController extends SubsystemBase {
  public static StateController _instance;

  @AutoLogOutput private RobotMode m_Mode;
  @AutoLogOutput private LevelState m_Level;

  public StateController() {
    setCoral();
  }

  public void setCoral() {
    m_Mode = RobotMode.CORAL;
  }

  public void setAlgae() {
    m_Mode = RobotMode.ALGAE;
  }

  public void setL1() {
    m_Level = LevelState.L1;
  }

  public void setL2() {
    m_Level = LevelState.L2;
  }

  public void setL3() {
    m_Level = LevelState.L3;
  }

  public void setL4() {
    m_Level = LevelState.L1;
  }

  public boolean isL1() {
    return m_Level == LevelState.L1;
  }

  public boolean isL2() {
    return m_Level == LevelState.L2;
  }

  public boolean isL3() {
    return m_Level == LevelState.L3;
  }

  public boolean isL4() {
    return m_Level == LevelState.L4;
  }

  public boolean isCoralMode() {
    return m_Mode == RobotMode.CORAL;
  }

  public boolean isAlgaeMode() {
    return m_Mode == RobotMode.ALGAE;
  }

  public boolean hasGamePiece(Manipulator manipulator) {
    return manipulator.hasGamePiece();
  }

  public RobotMode getMode() {
    return m_Mode;
  }

  public Command setCoralMode(Manipulator manipulator) {
    return run(
        () -> {
          setCoral();
          manipulator.setOpeningToCoral();
        });
  }

  public Command setAlgaeMode(Manipulator manipulator) {
    return run(
        () -> {
          setAlgae();
          manipulator.setOpeningToAlgae();
        });
  }
}
