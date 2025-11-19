// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.StateController;
import frc.robot.util.RobotState;

// "it will work first try" -person who speaks only lies
public class CustomBranchAuto extends SequentialCommandGroup {

  /** Creates a new CustomBranchAuto. */
  public CustomBranchAuto(StateController stateController) {

    // Add your commands in addCommands()
    addCommands(

        // set first state
        Commands.runOnce(() -> stateController.setWantedState(RobotState.HAS_PIECE_CORAL)),

        //// * BRANCH 1 * ////
        Commands.runOnce(() -> stateController.setBranch(stateController.getAutoBranch1())),

        // drive to branch 1
        // TODO: drive to branch auto align goes here when done

        // raise and set superstructure to L4
        Commands.runOnce(() -> stateController.setWantedState(RobotState.SCORE_L4)),

        // auto fire changes state to no_piece_coral automatically
        Commands.waitUntil(() -> stateController.getCurrentState() == RobotState.NO_PIECE_CORAL),

        // intake and drive to source
        Commands.parallel(
            Commands.runOnce(() -> stateController.setWantedState(RobotState.INTAKE_CORAL)) // ,
            // TODO: drive to source goes here when done
            ),
        Commands.waitUntil(() -> stateController.hasGamePiece()),

        //// * BRANCH 2 * ////
        Commands.runOnce(() -> stateController.setBranch(stateController.getAutoBranch2())),

        // drive to branch 2
        // TODO: drive to branch auto align goes here when done

        // raise and set superstructure to L4
        Commands.runOnce(() -> stateController.setWantedState(RobotState.SCORE_L4)),

        // auto fire changes state to no_piece_coral automatically
        Commands.waitUntil(() -> stateController.getCurrentState() == RobotState.NO_PIECE_CORAL),

        // intake and drive to source
        Commands.parallel(
            Commands.runOnce(() -> stateController.setWantedState(RobotState.INTAKE_CORAL)) // ,
            // TODO: drive to source goes here when done
            ),
        Commands.waitUntil(() -> stateController.hasGamePiece()),

        //// * BRANCH 3 * ////
        Commands.runOnce(() -> stateController.setBranch(stateController.getAutoBranch3())),

        // drive to branch 3
        // TODO: drive to branch auto align goes here when done

        // raise and set superstructure to L4
        Commands.runOnce(() -> stateController.setWantedState(RobotState.SCORE_L4)),

        // auto fire changes state to no_piece_coral automatically
        Commands.waitUntil(() -> stateController.getCurrentState() == RobotState.NO_PIECE_CORAL),

        // intake and drive to source
        Commands.parallel(
            Commands.runOnce(() -> stateController.setWantedState(RobotState.INTAKE_CORAL)) // ,
            // TODO: drive to source goes here when done
            ),
        Commands.waitUntil(() -> stateController.hasGamePiece()),

        //// * BRANCH 4 * ////
        Commands.runOnce(() -> stateController.setBranch(stateController.getAutoBranch3())),

        // drive to branch 4
        // TODO: drive to branch auto align goes here when done

        // raise and set superstructure to L4
        Commands.runOnce(() -> stateController.setWantedState(RobotState.SCORE_L4)),

        // auto fire changes state to no_piece_coral automatically
        Commands.waitUntil(() -> stateController.getCurrentState() == RobotState.NO_PIECE_CORAL),

        // intake and drive to source
        Commands.parallel(
            Commands.runOnce(() -> stateController.setWantedState(RobotState.INTAKE_CORAL)) // ,
            // TODO: drive to source goes here when done
            ),
        Commands.waitUntil(() -> stateController.hasGamePiece()));
  }
}
