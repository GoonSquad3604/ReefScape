// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos.CustomBranchAuto;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmIOPhoenixRev;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Climber.ClimberIOPhoenix;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorIONeo;
import frc.robot.subsystems.LED.LEDs;
import frc.robot.subsystems.Manipulator.Manipulator;
import frc.robot.subsystems.Manipulator.ManipulatorIOPhoenixRev;
import frc.robot.subsystems.StateController;
import frc.robot.subsystems.StateController.Branch;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.MitoCANdriaIO;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.LevelState;
import frc.robot.util.RobotState;
import java.util.Map;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  public final Drive drive;
  private final Vision vision;
  public final StateController stateController;
  private final SuperStructure superStructure;
  private final Arm arm;
  private final Manipulator manipulator;
  public final Climber climber;
  public final Elevator elevator;
  private final LEDs LED;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandJoystick operatorButtonBox = new CommandJoystick(1);
  private final CommandJoystick operatorReefBox = new CommandJoystick(2);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new MitoCANdriaIO(),
                new VisionIOPhotonVision(camera4Name, robotToCamera4),
                new VisionIOPhotonVision(camera1Name, robotToCamera1),
                new VisionIOPhotonVision(camera2Name, robotToCamera2),
                new VisionIOPhotonVision(camera3Name, robotToCamera3));
        manipulator = new Manipulator(new ManipulatorIOPhoenixRev());
        arm = new Arm(new ArmIOPhoenixRev());
        elevator = new Elevator(new ElevatorIONeo());
        climber = new Climber(new ClimberIOPhoenix());
        LED = new LEDs();
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new MitoCANdriaIO(),
                new VisionIOPhotonVisionSim(camera4Name, robotToCamera4, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));
        manipulator = new Manipulator(new ManipulatorIOPhoenixRev());
        arm = new Arm(new ArmIOPhoenixRev());
        elevator = new Elevator(new ElevatorIONeo());
        climber = new Climber(new ClimberIOPhoenix());
        LED = new LEDs();
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new MitoCANdriaIO(),
                new VisionIO() {},
                new VisionIO() {});
        manipulator = new Manipulator(new ManipulatorIOPhoenixRev());
        arm = new Arm(new ArmIOPhoenixRev());
        elevator = new Elevator(new ElevatorIONeo());
        climber = new Climber(new ClimberIOPhoenix());
        LED = new LEDs();
        break;
    }
    stateController = StateController.getInstance();
    superStructure = new SuperStructure(manipulator, arm, elevator, stateController);

    LED.setDefaultCommand(LED.defaultLeds(stateController.getCurrentState()));

    configureButtonBindings();

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoChooser.addOption("CustomBranchAuto", new CustomBranchAuto(stateController));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    /* Triggers */
    Trigger coralMode = new Trigger(() -> stateController.isCoralMode());
    Trigger algaeMode = new Trigger(() -> stateController.isAlgaeMode());
    Trigger climbMode = new Trigger(() -> stateController.isClimbMode());
    Trigger hasGamePiece = new Trigger(() -> stateController.hasGamePiece());

    BooleanSupplier slowMode = new Trigger(() -> driverController.getRightTriggerAxis() > 0.01);

    // Rumble controler for 1s when endgame begins
    new Trigger(() -> Timer.getMatchTime() <= 20 && Timer.getMatchTime() >= 18)
        .onTrue(
            Commands.runOnce(
                    () -> driverController.setRumble(GenericHID.RumbleType.kRightRumble, .8))
                .andThen(new WaitCommand(1))
                .andThen(() -> driverController.setRumble(GenericHID.RumbleType.kRightRumble, 0)));

    /* DRIVER BUTTONS */

    // Default drive command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            slowMode));

    // Coral mode + has piece -> angle towards reef panel
    // driverController
    //     .b()
    //     .and(coralMode)
    //     .and(hasGamePiece)
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -driverController.getLeftY(),
    //             () -> -driverController.getLeftX(),
    //             () -> AllianceFlipUtil.apply(drive.getClosestReefPanel()).getRotation()));

    // Coral mode + no piece -> angle towards source
    // driverController
    //     .b()
    //     .and(coralMode)
    //     .and(hasGamePiece.negate())
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -driverController.getLeftY(),
    //             () -> -driverController.getLeftX(),
    //             () -> AllianceFlipUtil.apply(drive.getClosestSource()).getRotation()));

    // Algae mode + has piece -> angle towards processor
    // driverController
    //     .b()
    //     .and(algaeMode)
    //     .and(hasGamePiece)
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -driverController.getLeftY(),
    //             () -> -driverController.getLeftX(),
    //             () ->
    // AllianceFlipUtil.apply(FieldConstants.Processor.centerFace).getRotation()));

    // Algae mode + no piece -> angle towards reef
    // driverController
    //     .b()
    //     .and(algaeMode)
    //     .and(hasGamePiece.negate())
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -driverController.getLeftY(),
    //             () -> -driverController.getLeftX(),
    //             () -> AllianceFlipUtil.apply(drive.getClosestReefPanel()).getRotation()));

    // Robot Relative Drive
    driverController
        .a()
        .whileTrue(
            DriveCommands.joystickRobotRelativeDrive(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX(),
                slowMode));

    // Switch to X pattern when X button is pressed (lock drive train)
    driverController.x().whileTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when start button is pressed
    driverController
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // Climb buttons (only while in climb mode)

    driverController
        .povUp()
        .or(driverController.povUpLeft().or(driverController.povUpRight()))
        .and(climbMode)
        .onTrue(climber.moveClimberUp());
    driverController
        .povUp()
        .or(driverController.povUpLeft().or(driverController.povUpRight()))
        .and(climbMode)
        .onFalse(climber.stop());

    driverController.povDown().and(climbMode).onTrue(climber.moveClimberDown());
    driverController.povDown().and(climbMode).onFalse(climber.stop());

    /* SOURCE PATHFINDS */

    // Left bumper, coral mode, no piece -> left source
    // driverController
    //     .leftBumper()
    //     .and(coralMode)
    //     .and(hasGamePiece.negate())
    //     .whileTrue(
    //         Commands.defer(
    //                 () -> AutoAline.autoAlineToPose(this, stateController.getSourcePose(true)),
    //                 Set.of(drive))
    //             .andThen(LED.strobeCommand(Color.kDarkOrange, .333)));

    // Right bumper, coral mode, no piece -> right source
    // driverController
    //     .rightBumper()
    //     .and(coralMode)
    //     .and(hasGamePiece.negate())
    //     .whileTrue(
    //         Commands.defer(
    //                 () -> AutoAline.autoAlineToPose(this, stateController.getSourcePose(false)),
    //                 Set.of(drive))
    //             .andThen(LED.strobeCommand(Color.kDarkOrange, .333)));

    // L/R Bumper drives to state controller branch (reef box)
    // driverController
    //     .leftBumper()
    //     .or(driverController.rightBumper())
    //     .and(coralMode)
    //     .and(hasGamePiece)
    //     .whileTrue(
    //         Commands.defer(
    //             () ->
    //                 AutoAline.autoAlineToPose(this, stateController.getBranch())
    //                     .andThen(Commands.runOnce(() -> stateController.setHathConcluded())),
    //             Set.of(drive)));

    /* ALGAE PATHFINDS */

    // Left bumper, algae mode, has piece -> barge
    // driverController
    //     .leftBumper()
    //     .and(algaeMode)
    //     .and(hasGamePiece)
    //     .and(gotGamePieceAutoAlgae.negate())
    //     .whileTrue(
    //         Commands.sequence(
    //             Commands.defer(() -> AutoAline.autoAlineToBarge(this), Set.of(drive)),
    //             Commands.runOnce(() -> stateController.setHathConcluded())));

    // Right bumper, algae mode, has piece -> processor
    // driverController
    //     .rightBumper()
    //     .and(algaeMode)
    //     .and(hasGamePiece)
    //     .and(gotGamePieceAutoAlgae.negate())
    //     .whileTrue(
    //         Commands.sequence(
    //             Commands.defer(() -> AutoAline.autoAlineToProcessorPose(this), Set.of(drive)),
    //             manipulator.shootAlgae(stateController.isL4())));

    // Right or Left bumper, algae mode, no game piece -> closest reef panel
    // driverController
    //     .rightBumper()
    //     .or(driverController.leftBumper())
    //     .and(algaeMode)
    //     .and(hasGamePiece.negate())
    //     .whileTrue(
    //         Commands.sequence(
    //             Commands.runOnce(() -> stateController.setGotPieceAutoAlgae()),
    //             Commands.defer(
    //                 () -> AutoAline.autoAlineToAlgaeReefPose(this, superStructure),
    //                 Set.of(drive))));

    // driverController
    //     .rightBumper()
    //     .or(driverController.leftBumper())
    //     .onFalse(Commands.runOnce(() -> stateController.setNotGotPieceAutoAlgae()));

    /* OPERATOR BUTTONS */

    // Set robot mode to coral
    operatorButtonBox
        .button(2)
        .onTrue(Commands.runOnce(() -> stateController.setWantedState(RobotState.NO_PIECE_CORAL)));

    // Set robot mode to algae
    operatorButtonBox
        .button(1)
        .onTrue(Commands.runOnce(() -> stateController.setWantedState(RobotState.NO_PIECE_ALGAE)));

    // Set robot mode to climb when button coral and algae are pressed
    operatorButtonBox
        .button(1)
        .and(operatorButtonBox.button(2))
        .onTrue(Commands.runOnce(() -> stateController.setWantedState(RobotState.CLIMB)));

    // Set Coral Level to L4
    operatorButtonBox.button(3).and(coralMode).onTrue(stateController.setL4());

    // Algae Barge (manual)
    operatorButtonBox
        .button(3)
        .and(algaeMode)
        .onTrue(Commands.runOnce(() -> stateController.setWantedState(RobotState.ALGAE_NET)));

    // Set Coral Level to L3
    operatorButtonBox.button(4).and(coralMode).onTrue(stateController.setL3());

    // Intake L3 Algae
    operatorButtonBox
        .button(4)
        .and(algaeMode)
        .onTrue(
            Commands.runOnce(
                () -> stateController.setWantedState(RobotState.INTAKE_ALGAE_REEF_L3)));

    // Set Coral Level to L2
    operatorButtonBox.button(5).and(coralMode).onTrue(stateController.setL2());

    // Intake L2 Algae
    operatorButtonBox
        .button(5)
        .and(algaeMode)
        .onTrue(
            Commands.runOnce(
                () -> stateController.setWantedState(RobotState.INTAKE_ALGAE_REEF_L2)));

    // L1 button + coral -> L1 coral position
    operatorButtonBox
        .button(6)
        .and(coralMode)
        .onTrue(Commands.runOnce(() -> stateController.setWantedState(RobotState.MANUAL_L1)));

    // L1 button + algae + game piece -> Processor position
    operatorButtonBox
        .button(6)
        .and(algaeMode)
        .and(hasGamePiece)
        .onTrue(Commands.runOnce(() -> stateController.setWantedState(RobotState.ALGAE_PROCESSOR)));

    // L1 button + algae + no game piece -> Lolipop intake
    operatorButtonBox
        .button(6)
        .and(algaeMode)
        .and(hasGamePiece.negate())
        .onTrue(
            Commands.runOnce(
                () -> stateController.setWantedState(RobotState.INTAKE_ALGAE_LOLIPOP)));

    // coral home
    operatorButtonBox
        .button(7)
        .and(coralMode)
        .onTrue(Commands.runOnce(() -> stateController.setWantedState(RobotState.NO_PIECE_CORAL)));

    // algae home (processor position)
    operatorButtonBox
        .button(7)
        .and(algaeMode)
        .onTrue(Commands.runOnce(() -> stateController.setWantedState(RobotState.NO_PIECE_ALGAE)));

    // button 8 (climb deploy) and button 9 (ggg) are now free under state based robot

    // Set arm to climb position
    // operatorButtonBox.button(8).and(climbMode).onTrue(arm.climb());

    // gogogadget INTAKE!!
    // operatorButtonBox
    //     .button(9)
    //     .and(intakeMode)
    //     .onTrue(Commands.runOnce(() -> stateController.setLongIntake()));

    // Intake button toggle (coral mode)
    operatorButtonBox
        .button(10)
        .and(coralMode)
        .and(hasGamePiece.negate())
        .onTrue(
            Commands.either(
                Commands.runOnce(() -> stateController.setWantedState(RobotState.NO_PIECE_CORAL)),
                Commands.runOnce(() -> stateController.setWantedState(RobotState.INTAKE_CORAL)),
                stateController::isIntakeMode));

    // Manual elevator
    operatorButtonBox
        .button(11)
        .onTrue(
            Commands.select(
                Map.ofEntries(
                    Map.entry(
                        LevelState.L1, // if L1 State
                        // Command at state l1
                        Commands.runOnce(
                            () -> stateController.setWantedState(RobotState.MANUAL_L1))),
                    Map.entry(
                        LevelState.L2, // if L2 State
                        // Command at state l2
                        Commands.runOnce(
                            () -> stateController.setWantedState(RobotState.MANUAL_L2))),
                    Map.entry(
                        LevelState.L3, // if L3 State
                        // Command at state l3
                        Commands.runOnce(
                            () -> stateController.setWantedState(RobotState.MANUAL_L3))),
                    Map.entry(
                        LevelState.L4, // if L4 State
                        // Command at state l4
                        Commands.runOnce(
                            () -> stateController.setWantedState(RobotState.MANUAL_L4)))),
                stateController::getLevel));

    // Fire button + coral mode -> set fire mode
    operatorButtonBox
        .button(12)
        .and(coralMode)
        .onTrue(Commands.runOnce(() -> stateController.setWantedState(RobotState.FIRE)));

    // fire button on false + coral mode -> home robot
    operatorButtonBox
        .button(12)
        .and(coralMode)
        .onFalse(Commands.runOnce(() -> stateController.setWantedState(RobotState.NO_PIECE_CORAL)));

    // fire button + algae mode -> set fire mode (power dependant on if robot is at L4)
    operatorButtonBox
        .button(12)
        .and(algaeMode)
        .onTrue(Commands.runOnce(() -> stateController.setWantedState(RobotState.FIRE)));

    // fire button on false + algae mode -> home JUST ELEVATOR
    operatorButtonBox
        .button(12)
        .and(algaeMode)
        .onFalse(Commands.runOnce(() -> stateController.setWantedState(RobotState.NO_PIECE_ALGAE)));

    /* REEF BOX (the reef is a lie) */
    operatorReefBox.button(12).onTrue(stateController.setBranch(Branch.FRONT_LEFTBRANCH));
    operatorReefBox.button(11).onTrue(stateController.setBranch(Branch.FRONT_RIGHTBRANCH));
    operatorReefBox.button(10).onTrue(stateController.setBranch(Branch.FRONTLEFT_LEFTBRANCH));
    operatorReefBox.button(9).onTrue(stateController.setBranch(Branch.FRONTLEFT_RIGHTBRANCH));
    operatorReefBox.button(8).onTrue(stateController.setBranch(Branch.BACKLEFT_LEFTBRANCH));
    operatorReefBox.button(7).onTrue(stateController.setBranch(Branch.BACKLEFT_RIGHTBRANCH));
    operatorReefBox.button(6).onTrue(stateController.setBranch(Branch.BACK_LEFTBRANCH));
    operatorReefBox.button(5).onTrue(stateController.setBranch(Branch.BACK_RIGHTBRANCH));
    operatorReefBox.button(4).onTrue(stateController.setBranch(Branch.BACKRIGHT_LEFTBRANCH));
    operatorReefBox.button(3).onTrue(stateController.setBranch(Branch.BACKRIGHT_RIGHTBRANCH));
    operatorReefBox.button(2).onTrue(stateController.setBranch(Branch.FRONTRIGHT_LEFTBRANCH));
    operatorReefBox.button(1).onTrue(stateController.setBranch(Branch.FRONTRIGHT_RIGHTBRANCH));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
