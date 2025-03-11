package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmConstants;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.Manipulator.Manipulator;
import frc.robot.subsystems.drive.Drive;
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

  private Manipulator manipulator;
  private Elevator elevator;
  private Arm arm;
  public Drive drive;

  @AutoLogOutput
  private boolean autoReadyFireIsTrue = false;


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

  public boolean stupid(Pose2d position1, Pose2d position2){
      if(Math.abs(position1.getX() - position2.getX()) < 0.1 
      && Math.abs(position1.getY() - position2.getY()) < 0.1
      && Math.abs(position1.getRotation().getRadians() - position2.getRotation().getRadians()) < 5 * (Math.PI / 180)){
        return true;
      }
    
    return false;
  }
  

  public boolean autoReadyFire(){
    Pose2d currentPosition = drive.getPose();
    boolean isInPosition = false;
    for(int i = 0; i < FieldConstants.Reef.leftRobotBranchPoses.size(); i++){
        if(stupid(currentPosition, FieldConstants.Reef.leftRobotBranchPoses.get(i))){
          isInPosition = true;
          break;
        }
      }
    if(!isInPosition){
      for(int i = 0; i < FieldConstants.Reef.rightRobotBranchPoses.size(); i++){
        if(stupid(currentPosition, FieldConstants.Reef.rightRobotBranchPoses.get(i))){
          isInPosition = true;
          break;
        }
      }
    }

    boolean armIsReady = false;
    boolean elevatorIsReady = false;
    if(isL1()){
      armIsReady = Math.abs(arm.getElbowPos() - ArmConstants.coralElbowL1) < 0.01 && Math.abs(arm.getWristPos() - ArmConstants.coralWristL1) < 0.01;
      elevatorIsReady = Math.abs(elevator.getPos() - ElevatorConstants.l1Pos) < 0.01;
    }
    else if(isL2()){
      armIsReady = Math.abs(arm.getElbowPos() - ArmConstants.coralElbowL2) < 0.01 && Math.abs(arm.getWristPos() - ArmConstants.coralWristL2) < 0.01;
      elevatorIsReady = Math.abs(elevator.getPos() - ElevatorConstants.l2Pos) < 0.01;
    }
    else if(isL3()){
      armIsReady = Math.abs(arm.getElbowPos() - ArmConstants.coralElbowL3) < 0.01 && Math.abs(arm.getWristPos() - ArmConstants.coralWristL3) < 0.01;
      elevatorIsReady = Math.abs(elevator.getPos() - ElevatorConstants.l3Pos) < 0.01;
    }
    else if(isL4()){
      armIsReady = Math.abs(arm.getElbowPos() - ArmConstants.coralElbowL4) < 0.01 && Math.abs(arm.getWristPos() - ArmConstants.coralWristL4) < 0.01;
      elevatorIsReady = Math.abs(elevator.getPos() - ElevatorConstants.l4Pos) < 0.01;
    }

    autoReadyFireIsTrue = isCoralMode() && manipulator.hasGamePiece() && !elevator.mahoming && isInPosition && armIsReady && elevatorIsReady;

    Logger.recordOutput("Robot Is In Position", isInPosition);
    Logger.recordOutput("Arm Is In Position", armIsReady);
    Logger.recordOutput("Elevator Is In Position", elevatorIsReady);

    return autoReadyFireIsTrue;

  }

  public void periodic(){
    Logger.recordOutput(" Left   Branch Positons ", FieldConstants.Reef.leftRobotBranchPoses.toArray(new Pose2d[0]));
    Logger.recordOutput("RightBranchPositons", FieldConstants.Reef.leftRobotBranchPoses.toArray(new Pose2d[0]));
  }

  public enum RobotTarget {
    REEF,
    PROCESSOR,
    SOURCE
  }
}
