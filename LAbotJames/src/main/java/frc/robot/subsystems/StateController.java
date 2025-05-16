package frc.robot.subsystems;

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
import frc.robot.util.RobotMode;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class StateController extends SubsystemBase {
  public static StateController _instance;

  @AutoLogOutput private RobotMode m_Mode;
  @AutoLogOutput private LevelState m_Level;
  @AutoLogOutput private Pose2d m_TragetPose;
  @AutoLogOutput private Intaking m_Intake;
  @AutoLogOutput private GoGoGadgetIntakeMode m_goGoGadgetIntake;
  @AutoLogOutput private ReefSide m_Side;
  @AutoLogOutput private Branch m_Branch;

  @AutoLogOutput private boolean autoReadyFireIsTrue = false;
  @AutoLogOutput private boolean armIsReady = false;
  @AutoLogOutput private boolean elevatorIsReady = false;
  @AutoLogOutput private boolean manipulatorIsReady = false;
  @AutoLogOutput private boolean autoAlineHathConcluded = false;

  @AutoLogOutput private boolean autoFireAutoOverride = false;

  private final LoggedDashboardChooser<Boolean> rightSourceChooser;
  private final LoggedDashboardChooser<Boolean> leftSourceChooser;

  public StateController() {
    m_Level = LevelState.MAHOME;
    m_Mode = RobotMode.IDLE;
    m_Intake = Intaking.NOINTAKE;
    m_Branch = Branch.FRONT_RIGHTBRANCH;
    m_goGoGadgetIntake = GoGoGadgetIntakeMode.SHORT;

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

  public void setCoral() {
    m_Mode = RobotMode.CORAL;
  }

  public void setAlgae() {
    m_Mode = RobotMode.ALGAE;
  }

  public void setClimb() {
    m_Mode = RobotMode.CLIMB;
  }

  public void setHathConcluded() {
    autoAlineHathConcluded = true;
  }

  public void setHathntConcluded() {
    autoAlineHathConcluded = false;
  }

  public boolean hathConcluded() {
    return autoAlineHathConcluded;
  }

  public void setIntake() {
    m_Intake = Intaking.INTAKE;
  }

  public void setNoIntake() {
    m_Intake = Intaking.NOINTAKE;
  }

  public void setLongIntake() {
    m_goGoGadgetIntake = GoGoGadgetIntakeMode.LONG;
  }

  public void setShortIntake() {
    m_goGoGadgetIntake = GoGoGadgetIntakeMode.SHORT;
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

  public boolean isIntakeMode() {
    return m_Intake == Intaking.INTAKE;
  }

  public boolean isLongIntakeMode() {
    return m_goGoGadgetIntake == GoGoGadgetIntakeMode.LONG;
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

  public ReefSide getSide() {
    return m_Side;
  }

  public Branch getBranch() {
    return m_Branch;
  }

  public Command setCoralMode() {
    return runOnce(
        () -> {
          setCoral();
        });
  }

  public Command setAlgaeMode() {
    return runOnce(
        () -> {
          setAlgae();
        });
  }

  public Command setClimbMode() {
    return runOnce(
        () -> {
          setClimb();
        });
  }

  public Command setIntakeMode() {
    return runOnce(
        () -> {
          setIntake();
        });
  }

  public Command setNoIntakeMode() {
    return runOnce(
        () -> {
          setNoIntake();
        });
  }

  public Command setBranch(Branch theBranch) {
    return runOnce(() -> m_Branch = theBranch);
  }

  public Command setOverride(boolean overrideSet) {
    return runOnce(() -> autoFireAutoOverride = overrideSet);
  }

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
        isCoralMode()
            && manipulator.hasGamePiece()
            && !elevator.mahoming
            && manipulatorIsReady
            && armIsReady
            && elevatorIsReady
            && (!DriverStation.isAutonomous() || autoFireAutoOverride);

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
        " Left   Branch Positons ",
        FieldConstants.Reef.leftRobotBranchPoses.toArray(new Pose2d[0]));
    Logger.recordOutput(
        "RightBranchPositons", FieldConstants.Reef.leftRobotBranchPoses.toArray(new Pose2d[0]));
    SmartDashboard.putBoolean("L4", isL4());
    SmartDashboard.putBoolean("L3", isL3());
    SmartDashboard.putBoolean("L2", isL2());
  }

  public enum Intaking {
    INTAKE,
    NOINTAKE
  }

  public enum GoGoGadgetIntakeMode {
    SHORT,
    LONG
  }

  public enum ReefSide {
    ONE,
    TWO,
    THREE,
    FOUR,
    FIVE,
    SIX
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
