package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmConstants;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.Manipulator.Manipulator;
import frc.robot.util.LevelState;
import frc.robot.util.RobotState;

public class StateController extends SubsystemBase {

  public static StateController _instance;

  // private enum WantedState {
  //   IDLE,
  //   STOPPED,
  //   INTAKE_CORAL,
  //   SCORE_L1,
  //   SCORE_L2,
  //   SCORE_L3,
  //   SCORE_L4,
  //   MANUAL_L4,
  //   MANUAL_L3,
  //   MANUAL_L2,
  //   MANUAL_L1,
  //   INTAKE_ALGAE_FROM_REEF,
  //   INTAKE_ALGAE_FROM_GROUND,
  //   INTAKE_ALGAE_FROM_LOLIPOP,
  //   SCORE_ALGAE_NET,
  //   SCORE_ALGAE_PROCESSOR,
  //   CLIMB
  // }

  // private enum CurrentState {
  //   IDLE,
  //   STOPPED,
  //   HAS_PIECE_CORAL,
  //   HAS_PIECE_ALGAE,
  //   INTAKE_CORAL,
  //   SCORE_L1,
  //   SCORE_L2,
  //   SCORE_L3,
  //   SCORE_L4,
  //   MANUAL_L4,
  //   MANUAL_L3,
  //   MANUAL_L2,
  //   MANUAL_L1,
  //   INTAKE_ALGAE_FROM_REEF,
  //   INTAKE_ALGAE_FROM_GROUND,
  //   INTAKE_ALGAE_FROM_LOLIPOP,
  //   SCORE_ALGAE_NET,
  //   SCORE_ALGAE_PROCESSOR,
  //   CLIMB
  // }

  private static RobotState wantedState;
  private static RobotState currentState;
  private static RobotState previousState;

  @AutoLogOutput private LevelState m_Level;
  @AutoLogOutput private Branch m_Branch;

  @AutoLogOutput private boolean autoReadyFireIsTrue = false;
  @AutoLogOutput private boolean armIsReady = false;
  @AutoLogOutput private boolean elevatorIsReady = false;
  @AutoLogOutput private boolean manipulatorIsReady = false;

  private final LoggedDashboardChooser<Boolean> rightSourceChooser;
  private final LoggedDashboardChooser<Boolean> leftSourceChooser;

  public StateController() {

    wantedState = RobotState.IDLE;
    currentState = RobotState.IDLE;

    m_Level = LevelState.MAHOME;
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
        || currentState == RobotState.SCORE_L1
        || currentState == RobotState.SCORE_L2
        || currentState == RobotState.SCORE_L3
        || currentState == RobotState.SCORE_L4;
  }
  public boolean isAlgaeMode() {
    return currentState == RobotState.HAS_PIECE_ALGAE
        || currentState == RobotState.NO_PIECE_ALGAE
        || currentState == RobotState.INTAKE_ALGAE_FROM_GROUND
        || currentState == RobotState.INTAKE_ALGAE_FROM_LOLIPOP
        || currentState == RobotState.INTAKE_ALGAE_FROM_REEF
        || currentState == RobotState.SCORE_ALGAE_NET
        || currentState == RobotState.SCORE_ALGAE_PROCESSOR;
  }

  public boolean isClimbMode() {
    return currentState == RobotState.CLIMB;
  }

  public boolean hasGamePiece() {
    return currentState == RobotState.HAS_PIECE_ALGAE
        || currentState == RobotState.HAS_PIECE_CORAL;
  }

  public RobotState getCurrentState() {
    return currentState;
  }

  public RobotState getWantedState() {
    return wantedState;
  }

  public void setWantedState(RobotState newState) {
    wantedState = newState;
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

  // public boolean hasGamePiece(Manipulator manipulator) {
  //   return manipulator.hasGamePiece();
  // }

  public LevelState getLevel() {
    return m_Level;
  }

  public Branch getBranch() {
    return m_Branch;
  }

  public Command setBranch(Branch theBranch) {
    return runOnce(() -> m_Branch = theBranch);
  }

  // abomination
  public boolean autoReadyFire(Arm arm, Elevator elevator, Manipulator manipulator) {

    armIsReady = false;
    elevatorIsReady = false;
    manipulatorIsReady = manipulator.isInPosition(m_Level);

    if (isL1()) {
      armIsReady =
          Math.abs(arm.getElbowPos() - ArmConstants.coralElbowL1) < 0.075
              && Math.abs(arm.getWristPos() - ArmConstants.coralWristL1) < 0.075;
      elevatorIsReady = false;

    } else if (isL2()) {
      armIsReady =
          Math.abs(arm.getElbowPos() - ArmConstants.coralElbowL2) < 0.075
              && Math.abs(arm.getWristPos() - ArmConstants.coralWristL2) < 0.02;
      elevatorIsReady = Math.abs(elevator.getPos() - ElevatorConstants.l2Pos) < 0.5;
    } else if (isL3()) {
      armIsReady =
          Math.abs(arm.getElbowPos() - ArmConstants.coralElbowL3) < 0.075
              && Math.abs(arm.getWristPos() - ArmConstants.coralWristL3) < 0.02;
      elevatorIsReady = Math.abs(elevator.getPos() - ElevatorConstants.l3Pos) < 0.5;
    } else if (isL4()) {
      armIsReady =
          Math.abs(arm.getElbowPos() - ArmConstants.coralElbowL4) < 0.075
              && Math.abs(arm.getWristPos() - ArmConstants.coralWristL4) < 0.075;
      elevatorIsReady = Math.abs(elevator.getPos() - ElevatorConstants.l4Pos) < 0.5;
    }

    autoReadyFireIsTrue =
        currentState == RobotState.HAS_PIECE_CORAL
            && manipulator.hasGamePiece()
            && !elevator.mahoming
            && manipulatorIsReady
            && armIsReady
            && elevatorIsReady
            && (!DriverStation.isAutonomous());

    return autoReadyFireIsTrue;
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
        "Left Branch Positons",
        FieldConstants.Reef.leftRobotBranchPoses.toArray(new Pose2d[0]));
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
