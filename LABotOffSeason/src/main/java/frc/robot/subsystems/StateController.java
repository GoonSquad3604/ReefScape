package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.util.LevelState;
import frc.robot.util.RobotState;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class StateController extends SubsystemBase {

  public static StateController _instance;

  // @AutoLogOutput private static RobotState wantedState;
  @AutoLogOutput private static RobotState currentState;
  @AutoLogOutput private static RobotState previousState;

  @AutoLogOutput private LevelState m_Level;
  @AutoLogOutput private Branch m_Branch;

  private final LoggedDashboardChooser<Boolean> rightSourceChooser;
  private final LoggedDashboardChooser<Boolean> leftSourceChooser;

  public StateController() {

    // wantedState = RobotState.IDLE;
    currentState = RobotState.IDLE;

    m_Level = LevelState.HOME;
    m_Branch = Branch.FRONT_RIGHTBRANCH;

    rightSourceChooser = new LoggedDashboardChooser<>("Right Source NearOrFar");
    rightSourceChooser.addOption("Near", true);
    rightSourceChooser.addOption("Far", false);
    rightSourceChooser.addDefaultOption("Near", true);

    leftSourceChooser = new LoggedDashboardChooser<>("Left Source NearOrFar");
    leftSourceChooser.addOption("Near", true);
    leftSourceChooser.addOption("Far", false);
    leftSourceChooser.addDefaultOption("Near", true);
  }

  public static StateController getInstance() {
    if (_instance == null) {
      _instance = new StateController();
    }
    return _instance;
  }

  public boolean isCoralMode() {
    return currentState == RobotState.HAS_PIECE_CORAL
        || currentState == RobotState.NO_PIECE_CORAL
        || currentState == RobotState.INTAKE_CORAL
        || currentState == RobotState.MANUAL_L1
        || currentState == RobotState.MANUAL_L2
        || currentState == RobotState.MANUAL_L3
        || currentState == RobotState.MANUAL_L4
        // || currentState == RobotState.SCORE_L1
        || currentState == RobotState.SCORE_L2
        || currentState == RobotState.SCORE_L3
        || currentState == RobotState.SCORE_L4;
  }

  public boolean isAlgaeMode() {
    return currentState == RobotState.HAS_PIECE_ALGAE
        || currentState == RobotState.NO_PIECE_ALGAE
        || currentState == RobotState.INTAKE_ALGAE_GROUND
        || currentState == RobotState.INTAKE_ALGAE_LOLIPOP
        || currentState == RobotState.INTAKE_ALGAE_REEF_L2
        || currentState == RobotState.INTAKE_ALGAE_REEF_L3
        || currentState == RobotState.ALGAE_NET
        || currentState == RobotState.ALGAE_PROCESSOR;
  }

  public boolean isClimbMode() {
    return currentState == RobotState.CLIMB;
  }

  public boolean hasGamePiece() {
    return currentState == RobotState.HAS_PIECE_ALGAE || currentState == RobotState.HAS_PIECE_CORAL;
  }

  public boolean isIntakeMode() {
    return currentState == RobotState.INTAKE_CORAL;
  }

  public RobotState getCurrentState() {
    return currentState;
  }

  // public RobotState getWantedState() {
  //   return wantedState;
  // }

  public RobotState getPreviousState() {
    return previousState;
  }

  public void setWantedState(RobotState newState) {
    previousState = currentState;
    currentState = newState;
  }

  public Command setHome() {
    return runOnce(() -> m_Level = LevelState.HOME);
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

  public boolean isHome() {
    return m_Level == LevelState.HOME;
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

  public LevelState getLevel() {
    return m_Level;
  }

  public Branch getBranch() {
    return m_Branch;
  }

  public Command setBranch(Branch theBranch) {
    return runOnce(() -> m_Branch = theBranch);
  }

  public Pose2d getSourcePose(boolean isLeft) {
    boolean isRightNear = rightSourceChooser.get();
    boolean isLeftNear = leftSourceChooser.get();

    if (isLeft) {
      if (isLeftNear) {
        return FieldConstants.CoralStation.leftNearIntakePos;
      } else {
        return FieldConstants.CoralStation.leftFarIntakePos;
      }
    } else {
      if (isRightNear) {
        return FieldConstants.CoralStation.rightNearIntakePos;
      } else {
        return FieldConstants.CoralStation.rightFarIntakePos;
      }
    }
  }

  public void periodic() {
    Logger.recordOutput(
        "Left Branch Positons", FieldConstants.Reef.leftRobotBranchPoses.toArray(new Pose2d[0]));
    Logger.recordOutput(
        "Right Branch Positons", FieldConstants.Reef.leftRobotBranchPoses.toArray(new Pose2d[0]));
    SmartDashboard.putBoolean("L4", isL4());
    SmartDashboard.putBoolean("L3", isL3());
    SmartDashboard.putBoolean("L2", isL2());
  }

  public enum Branch {
    BACK_LEFTBRANCH,
    BACK_RIGHTBRANCH,
    BACKLEFT_LEFTBRANCH,
    BACKLEFT_RIGHTBRANCH,
    BACKRIGHT_LEFTBRANCH,
    BACKRIGHT_RIGHTBRANCH,
    FRONT_LEFTBRANCH,
    FRONT_RIGHTBRANCH,
    FRONTLEFT_LEFTBRANCH,
    FRONTLEFT_RIGHTBRANCH,
    FRONTRIGHT_LEFTBRANCH,
    FRONTRIGHT_RIGHTBRANCH
  }
}
