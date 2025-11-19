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

  private final LoggedDashboardChooser<Branch> autoBranchSelector1;
  private final LoggedDashboardChooser<Branch> autoBranchSelector2;
  private final LoggedDashboardChooser<Branch> autoBranchSelector3;
  private final LoggedDashboardChooser<Branch> autoBranchSelector4;

  public StateController() {

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

    autoBranchSelector1 = new LoggedDashboardChooser<>("FIRST BRANCH");
    autoBranchSelector2 = new LoggedDashboardChooser<>("SECOND BRANCH");
    autoBranchSelector3 = new LoggedDashboardChooser<>("THIRD BRANCH");
    autoBranchSelector4 = new LoggedDashboardChooser<>("FOURTH BRANCH");
    addAutoBranchOptions();
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

  public static StateController getInstance() {
    if (_instance == null) {
      _instance = new StateController();
    }
    return _instance;
  }

  private void addAutoBranchOptions() {

    // this might either be the worst or best code ever written.
    // default of each is the 3/4 piece for right
    autoBranchSelector1.addOption("↖L", Branch.BACKLEFT_LEFTBRANCH);
    autoBranchSelector1.addOption("↖R", Branch.BACKLEFT_RIGHTBRANCH);
    autoBranchSelector1.addOption("↙L", Branch.FRONTLEFT_LEFTBRANCH);
    autoBranchSelector1.addOption("↙R", Branch.FRONTLEFT_RIGHTBRANCH);
    autoBranchSelector1.addOption("↓L", Branch.FRONT_LEFTBRANCH);
    autoBranchSelector1.addOption("↓R", Branch.FRONT_RIGHTBRANCH);
    autoBranchSelector1.addOption("↘L", Branch.FRONTRIGHT_LEFTBRANCH);
    autoBranchSelector1.addOption("↘R", Branch.FRONTRIGHT_RIGHTBRANCH);
    autoBranchSelector1.addOption("↗L", Branch.BACKRIGHT_LEFTBRANCH);
    autoBranchSelector1.addOption("↗R", Branch.BACKRIGHT_RIGHTBRANCH);
    autoBranchSelector1.addOption("↑L", Branch.BACK_LEFTBRANCH);
    autoBranchSelector1.addOption("↑R", Branch.BACK_RIGHTBRANCH);
    autoBranchSelector1.addDefaultOption("↗L", Branch.BACKRIGHT_LEFTBRANCH);

    autoBranchSelector2.addOption("↖L", Branch.BACKLEFT_LEFTBRANCH);
    autoBranchSelector2.addOption("↖R", Branch.BACKLEFT_RIGHTBRANCH);
    autoBranchSelector2.addOption("↙L", Branch.FRONTLEFT_LEFTBRANCH);
    autoBranchSelector2.addOption("↙R", Branch.FRONTLEFT_RIGHTBRANCH);
    autoBranchSelector2.addOption("↓L", Branch.FRONT_LEFTBRANCH);
    autoBranchSelector2.addOption("↓R", Branch.FRONT_RIGHTBRANCH);
    autoBranchSelector2.addOption("↘L", Branch.FRONTRIGHT_LEFTBRANCH);
    autoBranchSelector2.addOption("↘R", Branch.FRONTRIGHT_RIGHTBRANCH);
    autoBranchSelector2.addOption("↗L", Branch.BACKRIGHT_LEFTBRANCH);
    autoBranchSelector2.addOption("↗R", Branch.BACKRIGHT_RIGHTBRANCH);
    autoBranchSelector2.addOption("↑L", Branch.BACK_LEFTBRANCH);
    autoBranchSelector2.addOption("↑R", Branch.BACK_RIGHTBRANCH);
    autoBranchSelector2.addDefaultOption("↘R", Branch.FRONTRIGHT_RIGHTBRANCH);

    autoBranchSelector3.addOption("↖L", Branch.BACKLEFT_LEFTBRANCH);
    autoBranchSelector3.addOption("↖R", Branch.BACKLEFT_RIGHTBRANCH);
    autoBranchSelector3.addOption("↙L", Branch.FRONTLEFT_LEFTBRANCH);
    autoBranchSelector3.addOption("↙R", Branch.FRONTLEFT_RIGHTBRANCH);
    autoBranchSelector3.addOption("↓L", Branch.FRONT_LEFTBRANCH);
    autoBranchSelector3.addOption("↓R", Branch.FRONT_RIGHTBRANCH);
    autoBranchSelector3.addOption("↘L", Branch.FRONTRIGHT_LEFTBRANCH);
    autoBranchSelector3.addOption("↘R", Branch.FRONTRIGHT_RIGHTBRANCH);
    autoBranchSelector3.addOption("↗L", Branch.BACKRIGHT_LEFTBRANCH);
    autoBranchSelector3.addOption("↗R", Branch.BACKRIGHT_RIGHTBRANCH);
    autoBranchSelector3.addOption("↑L", Branch.BACK_LEFTBRANCH);
    autoBranchSelector3.addOption("↑R", Branch.BACK_RIGHTBRANCH);
    autoBranchSelector3.addDefaultOption("↘L", Branch.FRONTRIGHT_LEFTBRANCH);

    autoBranchSelector4.addOption("↖L", Branch.BACKLEFT_LEFTBRANCH);
    autoBranchSelector4.addOption("↖R", Branch.BACKLEFT_RIGHTBRANCH);
    autoBranchSelector4.addOption("↙L", Branch.FRONTLEFT_LEFTBRANCH);
    autoBranchSelector4.addOption("↙R", Branch.FRONTLEFT_RIGHTBRANCH);
    autoBranchSelector4.addOption("↓L", Branch.FRONT_LEFTBRANCH);
    autoBranchSelector4.addOption("↓R", Branch.FRONT_RIGHTBRANCH);
    autoBranchSelector4.addOption("↘L", Branch.FRONTRIGHT_LEFTBRANCH);
    autoBranchSelector4.addOption("↘R", Branch.FRONTRIGHT_RIGHTBRANCH);
    autoBranchSelector4.addOption("↗L", Branch.BACKRIGHT_LEFTBRANCH);
    autoBranchSelector4.addOption("↗R", Branch.BACKRIGHT_RIGHTBRANCH);
    autoBranchSelector4.addOption("↑L", Branch.BACK_LEFTBRANCH);
    autoBranchSelector4.addOption("↑R", Branch.BACK_RIGHTBRANCH);
    autoBranchSelector4.addDefaultOption("↓R", Branch.FRONT_RIGHTBRANCH);
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

  public Branch getAutoBranch1() {
    return autoBranchSelector1.get();
  }

  public Branch getAutoBranch2() {
    return autoBranchSelector2.get();
  }

  public Branch getAutoBranch3() {
    return autoBranchSelector3.get();
  }

  public Branch getAutoBranch4() {
    return autoBranchSelector4.get();
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
