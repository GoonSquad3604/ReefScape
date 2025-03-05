package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
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
  @AutoLogOutput private RobotTarget m_Target;
  @AutoLogOutput private Pose2d m_TragetPose;

  public StateController() {
    // setCoral();
    m_Level = LevelState.MAHOME;
    m_Mode = RobotMode.IDLE;
  }

  public void setCoral() {
    m_Mode = RobotMode.CORAL;
  }

  public void setAlgae() {
    m_Mode = RobotMode.ALGAE;
  }

  public void setClimb() {
    m_Mode = RobotMode.CLIMB;
  }

  public Command setL1() {
    return runOnce(() -> m_Level = LevelState.L1);
  }

  public Command setL2() {
    return runOnce(() -> m_Level = LevelState.L2);
  }

  public Command setL3() {
    return runOnce(() -> m_Level = LevelState.L3);
  }

  public Command setL4() {
    return runOnce(() -> m_Level = LevelState.L4);
  }

  public Command setMahome() {
    return runOnce(() -> m_Level = LevelState.MAHOME);
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

  public boolean isMahome() {
    return m_Level == LevelState.MAHOME;
  }

  public boolean isCoralMode() {
    return m_Mode == RobotMode.CORAL;
  }

  public boolean isAlgaeMode() {
    return m_Mode == RobotMode.ALGAE;
  }

  public boolean isClimbMode() {
    return m_Mode == RobotMode.CLIMB;
  }

  public boolean hasGamePiece(Manipulator manipulator) {
    return manipulator.hasGamePiece();
  }

  public RobotMode getMode() {
    return m_Mode;
  }

  public LevelState getLevel() {
    return m_Level;
  }

  public Command setCoralMode(Manipulator manipulator) {
    return run(
        () -> {
          setCoral();
        });
  }

  public Command setAlgaeMode(Manipulator manipulator) {
    return run(
        () -> {
          setAlgae();
        });
  }

  public Command setClimbMode(Manipulator manipulator) {
    return run(
        () -> {
          setClimb();
        });
  }

  public enum RobotTarget {
    REEF,
    PROCESSOR,
    SOURCE
  }
}
