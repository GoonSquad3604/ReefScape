// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.Elevator.ElevatorIONeo;
import frc.robot.subsystems.StateController;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorToSetpoint extends Command {
  private LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG", 0.0);
  private LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS", 0.0);
  private LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/kV", 0.0);
  private LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/kA", 0.0);

  private Elevator elevator;
  private ElevatorIONeo io;

  private double userGoal;

  private ElevatorFeedforward feedforward;

  private TrapezoidProfile profile;
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
  private boolean mahoming;
  private boolean goesToState;
  private StateController state;
  // code
  // code
  // more code
  // wow even more code

  public ElevatorToSetpoint(
      Elevator elevator, double userGoal, boolean mahoming) { // booleon, booleoff -drew

    this.elevator = elevator;
    this.userGoal = userGoal;
    this.mahoming = mahoming;
    goesToState = false; // muy falso
    feedforward =
        new ElevatorFeedforward(
            ElevatorConstants.ks, ElevatorConstants.kg, ElevatorConstants.kv, ElevatorConstants.ka);

    addRequirements(elevator);
  }

  public ElevatorToSetpoint(Elevator elevator, double userGoal) { // booleon, booleoff -drew

    this.elevator = elevator;
    this.userGoal = userGoal;
    this.mahoming = false;
    goesToState = false; // muy falso
    feedforward =
        new ElevatorFeedforward(
            ElevatorConstants.ks, ElevatorConstants.kg, ElevatorConstants.kv, ElevatorConstants.ka);

    addRequirements(elevator);
  }

  public ElevatorToSetpoint(
      Elevator elevator, StateController state) { // Goes to target state if no goal is given

    this.elevator = elevator;
    this.userGoal = userGoal;
    this.mahoming = false;
    goesToState = true;
    this.state = state;

    feedforward =
        new ElevatorFeedforward(
            ElevatorConstants.ks, ElevatorConstants.kg, ElevatorConstants.kv, ElevatorConstants.ka);

    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(5, 1.0));
    if (goesToState) {
      switch (state.getLevel()) {
        case L1:
          goal = new TrapezoidProfile.State((ElevatorConstants.l1Pos), 0);
          break;
        case L2:
          goal = new TrapezoidProfile.State((ElevatorConstants.l2Pos), 0);
          break;
        case L3:
          goal = new TrapezoidProfile.State((ElevatorConstants.l3Pos), 0);
          break;
        case L4:
          goal = new TrapezoidProfile.State((ElevatorConstants.l4Pos), 0);
          break;
        default:
          goal = new TrapezoidProfile.State(userGoal, 0);
      }

    } else {
      goal = new TrapezoidProfile.State(userGoal, 0);
    }
    setpoint = new TrapezoidProfile.State(elevator.getPos(), 0);
    elevator.mahoming = this.mahoming;
  }

  @Override
  public void execute() {

    // Retrieve profiled setpoint for the next0 timestep.
    // This setpoint moves toward the goal while obeying the constraints.

    setpoint = profile.calculate(0.2, setpoint, goal);

    // Send setpoint to offboard controller PID
    Logger.recordOutput("Set FeedForward", feedforward.calculate(setpoint.velocity));
    Logger.recordOutput("Setpoint Velocity", setpoint.velocity);
    elevator.setPosWff(setpoint.position, feedforward.calculate(setpoint.velocity));

    Logger.recordOutput("Desired motion", setpoint.position);
  }

  @Override
  public void end(boolean interrupted) {
    // elevator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
