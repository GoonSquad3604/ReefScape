package frc.robot.subsystems;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
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

public class StateController extends SubsystemBase {
  public static StateController _instance;

  @AutoLogOutput private RobotMode m_Mode;
  @AutoLogOutput private LevelState m_Level;
  @AutoLogOutput private RobotTarget m_Target;
  @AutoLogOutput private Pose2d m_TragetPose;
  @AutoLogOutput private Intaking m_Intake;
  @AutoLogOutput private ReefSide m_Side;
  @AutoLogOutput private LeftOrRight m_LeftOrRight;
  @AutoLogOutput private Branch m_Branch;

  // private Manipulator manipulator;
  // private Elevator elevator;
  // private Arm arm;
  // public Drive drive;

  @AutoLogOutput private boolean autoReadyFireIsTrue = false;
  @AutoLogOutput private boolean armIsReady = false;
  @AutoLogOutput private boolean elevatorIsReady = false;
  @AutoLogOutput private boolean manipulatorIsReady = false;

  public StateController() {
    m_Level = LevelState.MAHOME;
    m_Mode = RobotMode.IDLE;
    m_Intake = Intaking.NOINTAKE;
    m_Side = ReefSide.ONE;
    m_LeftOrRight = LeftOrRight.LEFT;
    m_Branch = Branch.FRONT_RIGHTBRANCH;
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

  public void setIntake() {
    m_Intake = Intaking.INTAKE;
  }

  public void setNoIntake() {
    m_Intake = Intaking.NOINTAKE;
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

  public Command setSide1() {
    return runOnce(() -> m_Side = ReefSide.ONE);
  }

  public Command setSide2() {
    return runOnce(() -> m_Side = ReefSide.TWO);
  }

  public Command setSide3() {
    return runOnce(() -> m_Side = ReefSide.THREE);
  }

  public Command setSide4() {
    return runOnce(() -> m_Side = ReefSide.FOUR);
  }

  public boolean isSide1() {
    return m_Side == ReefSide.ONE;
  }

  public boolean isSide2() {
    return m_Side == ReefSide.TWO;
  }

  public boolean isSide3() {
    return m_Side == ReefSide.THREE;
  }

  public boolean isSide4() {
    return m_Side == ReefSide.FOUR;
  }

  public boolean isSide5() {
    return m_Side == ReefSide.FIVE;
  }

  public boolean isSide6() {
    return m_Side == ReefSide.SIX;
  }

  public boolean isLeft() {
    return m_LeftOrRight == LeftOrRight.LEFT;
  }

  public boolean isRight() {
    return m_LeftOrRight == LeftOrRight.RIGHt;
  }

  public Command setSide5() {
    return runOnce(() -> m_Side = ReefSide.FIVE);
  }

  public Command setSide6() {
    return runOnce(() -> m_Side = ReefSide.SIX);
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

  public LeftOrRight getLeftOrRight() {
    return m_LeftOrRight;
  }

  public Branch getBranch() {
    return m_Branch;
  }

  public Command setCoralMode(Manipulator manipulator) {
    return runOnce(
        () -> {
          setCoral();
        });
  }

  public Command setAlgaeMode(Manipulator manipulator) {
    return runOnce(
        () -> {
          setAlgae();
        });
  }

  public Command setClimbMode(Manipulator manipulator) {
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

  public boolean stupid(Pose2d position1, Pose2d position2) {
    if (Math.abs(position1.getX() - position2.getX()) < 0.1
        && Math.abs(position1.getY() - position2.getY()) < 0.1
        && Math.abs(position1.getRotation().getRadians() - position2.getRotation().getRadians())
            < 5 * (Math.PI / 180)) {
      return true;
    }

    return false;
  }

  public Command setBranch(Branch theBranch) {
    return runOnce(() -> m_Branch = theBranch);
  }

  public boolean autoReadyFire(Arm arm, Elevator elevator, Manipulator manipulator) {
    // Pose2d currentPosition = drive.getPose();
    // boolean isInPosition = false;
    // for (int i = 0; i < FieldConstants.Reef.leftRobotBranchPoses.size(); i++) {
    //   if (stupid(currentPosition, FieldConstants.Reef.leftRobotBranchPoses.get(i))) {
    //     isInPosition = true;
    //     break;
    //   }
    // }
    // if (!isInPosition) {
    //   for (int i = 0; i < FieldConstants.Reef.rightRobotBranchPoses.size(); i++) {
    //     if (stupid(currentPosition, FieldConstants.Reef.rightRobotBranchPoses.get(i))) {
    //       isInPosition = true;
    //       break;
    //     }
    //   }
    // }

    armIsReady = false;
    elevatorIsReady = false;
    manipulatorIsReady = manipulator.isInPosition(m_Level);

    if (isL1()) {
      armIsReady =
          Math.abs(arm.getElbowPos() - ArmConstants.coralElbowL1) < 0.075
              && Math.abs(arm.getWristPos() - ArmConstants.coralWristL1) < 0.075;
      elevatorIsReady = Math.abs(elevator.getPos() - ElevatorConstants.l1Pos) < 0.50001;
    } else if (isL2()) {
      armIsReady =
          Math.abs(arm.getElbowPos() - ArmConstants.coralElbowL2) < 0.075
              && Math.abs(arm.getWristPos() - ArmConstants.coralWristL2) < 0.075;
      elevatorIsReady = Math.abs(elevator.getPos() - ElevatorConstants.l2Pos) < 0.500015;
    } else if (isL3()) {
      armIsReady =
          Math.abs(arm.getElbowPos() - ArmConstants.coralElbowL3) < 0.075
              && Math.abs(arm.getWristPos() - ArmConstants.coralWristL3) < 0.075;
      elevatorIsReady = Math.abs(elevator.getPos() - ElevatorConstants.l3Pos) < 0.50001;
    } else if (isL4()) {
      armIsReady =
          Math.abs(arm.getElbowPos() - ArmConstants.coralElbowL4) < 0.075
              && Math.abs(arm.getWristPos() - ArmConstants.coralWristL4) < 0.075000001;
      elevatorIsReady = Math.abs(elevator.getPos() - ElevatorConstants.l4Pos) < 0.50001;
    }

    autoReadyFireIsTrue =
        isCoralMode()
            && manipulator.hasGamePiece()
            && !elevator.mahoming
            && manipulatorIsReady
            && armIsReady
            && elevatorIsReady;

    // Logger.recordOutput("laserCanSees", manipulator);
    // Logger.recordOutput("ArmIsInPosition", armIsReady);
    // Logger.recordOutput("ElevatorIsInPosition", elevatorIsReady);

    return autoReadyFireIsTrue;
  }

  public PathPlannerPath getSourcePath(boolean isLeft) {

    try {
      if (isLeft) {
        return PathPlannerPath.fromPathFile("LeftSource");
      } else {
        return PathPlannerPath.fromPathFile("RightSource");
      }
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return null;
    }
  }

  public PathPlannerPath getAlgaePath(boolean processor) {
    try {
      if (processor) {
        return PathPlannerPath.fromPathFile("Processor");
      } else {
        return null;
      }
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return null;
    }
  }

  public void periodic() {
    Logger.recordOutput(
        " Left   Branch Positons ",
        FieldConstants.Reef.leftRobotBranchPoses.toArray(new Pose2d[0]));
    Logger.recordOutput(
        "RightBranchPositons", FieldConstants.Reef.leftRobotBranchPoses.toArray(new Pose2d[0]));
  }

  public enum RobotTarget {
    REEF,
    PROCESSOR,
    SOURCE
  }

  public enum Intaking {
    INTAKE,
    NOINTAKE
  }

  public enum ReefSide {
    ONE,
    TWO,
    THREE,
    FOUR,
    FIVE,
    SIX
  }

  public enum LeftOrRight {
    LEFT,
    RIGHt
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
