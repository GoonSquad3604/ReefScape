// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Manipulator.Manipulator;
import frc.robot.subsystems.StateController;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.drive.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreePieceRight extends SequentialCommandGroup {
  /** Creates a new ThreePieceRight. */
  public ThreePieceRight(
      StateController stateController,
      Elevator elevator,
      Manipulator manipulator,
      Arm arm,
      Drive drive,
      RobotContainer robotContainer,
      SuperStructure superStructure) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // stateController.setWantedState(RobotState.HAS_PIECE_CORAL),
        // stateController.setL4(),
        // stateController.setBranch(Branch.BACKRIGHT_LEFTBRANCH),
        // Commands.defer(
        //     () ->
        //         AutoAline.autoAlineToPose(robotContainer, stateController.getBranch())
        //             .andThen(Commands.runOnce(() -> stateController.setHathConcluded())),
        //     Set.of(drive)),

        // Commands.waitUntil(() -> !manipulator.hasGamePiece())
        //     .withName("waitingForManipulator")
        //     .withTimeout(2)
        //     .andThen(new ElevatorToSetpoint(elevator, ElevatorConstants.homePos, true))
        //     .until(() -> !elevator.mahoming)
        //     .andThen(elevator.runOnce(() -> elevator.stop())),

        // Commands.waitUntil(() -> !manipulator.gamePieceDetected()),
        // Commands.waitUntil(() -> elevator.getPos() < 14).withName("waitingForElevator"),
        // stateController.setBranch(Branch.FRONTRIGHT_RIGHTBRANCH),
        // Commands.either(
        //     Commands.defer(
        //         () ->
        //             AutoAline.autoAlineToPose(robotContainer, stateController.getBranch())
        //                 .andThen(Commands.runOnce(() -> stateController.setHathConcluded())),
        //         Set.of(drive)),
        //     Commands.defer(
        //         () ->
        //             AutoAline.autoAlineToPose(robotContainer,
        // stateController.getSourcePose(false)),
        //         Set.of(drive)),
        //     () -> manipulator.hasGamePiece()),
        // stateController.setIntakeMode(),
        // Commands.waitUntil(() -> !stateController.isIntakeMode()).withTimeout(2.999),
        // stateController.setAlgaeMode(),
        // Commands.defer(
        //     () ->
        //         AutoAline.autoAlineToPose(robotContainer, stateController.getBranch())
        //             .andThen(Commands.runOnce(() -> stateController.setHathConcluded())),
        //     Set.of(drive)));
        );
  }
}
