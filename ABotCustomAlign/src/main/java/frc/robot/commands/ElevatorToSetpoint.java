// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorToSetpoint extends Command {

  private Elevator elevator;

  private double userGoal;

  private ElevatorFeedforward feedforward;

  private TrapezoidProfile profile;
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
  private boolean mahoming;
  private boolean isAlgee;

  // end goal, homing check
  public ElevatorToSetpoint(Elevator elevator, double userGoal, boolean mahoming) {

    this.elevator = elevator;
    this.userGoal = userGoal;
    this.mahoming = mahoming;
    isAlgee = false;
    feedforward =
        new ElevatorFeedforward(
            ElevatorConstants.ks, ElevatorConstants.kg, ElevatorConstants.kv, ElevatorConstants.ka);

    addRequirements(elevator);
  }

  // end goal, homing check, algae check
  public ElevatorToSetpoint(
      Elevator elevator,
      double userGoal,
      boolean mahoming,
      boolean algee) { // booleon, booleoff -drew

    this.elevator = elevator;
    this.userGoal = userGoal;
    this.mahoming = mahoming;
    isAlgee = algee;
    feedforward =
        new ElevatorFeedforward(
            ElevatorConstants.ks, ElevatorConstants.kg, ElevatorConstants.kv, ElevatorConstants.ka);

    addRequirements(elevator);
  }

  // just end goal
  public ElevatorToSetpoint(Elevator elevator, double userGoal) {

    this.elevator = elevator;
    this.userGoal = userGoal;
    this.mahoming = false;
    isAlgee = false;
    feedforward =
        new ElevatorFeedforward(
            ElevatorConstants.ks, ElevatorConstants.kg, ElevatorConstants.kv, ElevatorConstants.ka);

    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (!isAlgee) profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(5, 2)); // 5, 1
    else profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(5, 1));

    goal = new TrapezoidProfile.State(userGoal, 0);

    setpoint = new TrapezoidProfile.State(elevator.getPos(), 0);
    elevator.mahoming = this.mahoming;
  }

  @Override
  public void execute() {

    // Retrieve profiled setpoint for the next timestep.
    // This setpoint moves toward the goal while obeying the constraints.

    setpoint = profile.calculate(0.2, setpoint, goal);

    // Send setpoint to offboard controller PID
    Logger.recordOutput("Set FeedForward", feedforward.calculate(setpoint.velocity));
    Logger.recordOutput("Setpoint Velocity", setpoint.velocity);
    elevator.setPosWff(setpoint.position, feedforward.calculate(setpoint.velocity));

    Logger.recordOutput("Desired motion", setpoint.position);
  }

  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
