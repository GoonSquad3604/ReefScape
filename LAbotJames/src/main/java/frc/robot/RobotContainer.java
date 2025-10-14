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
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorToSetpoint;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmIOPhoenixRev;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Climber.ClimberIOPhoenix;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.Elevator.ElevatorIONeo;
import frc.robot.subsystems.LED.LEDs;
import frc.robot.subsystems.Manipulator.Manipulator;
import frc.robot.subsystems.Manipulator.ManipulatorConstants;
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
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LevelState;
import java.util.Map;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
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
  private final LEDs lED;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandJoystick operatorButtonBox = new CommandJoystick(1);
  private final CommandJoystick operatorReefBox = new CommandJoystick(2);
  // private final CommandXboxController testController = new CommandXboxController(3);
  private final Supplier<Translation2d> joystickSupplier =
      () -> new Translation2d(driverController.getLeftY(), driverController.getLeftX());

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
        lED = new LEDs();
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
        lED = new LEDs();
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
        lED = new LEDs();
        break;
    }
    stateController = StateController.getInstance();
    superStructure = new SuperStructure(manipulator, arm, elevator);

    lED.setDefaultCommand(
        lED.defaultLeds(
            () -> stateController.getMode(),
            () -> stateController.isIntakeMode(),
            () -> stateController.hathConcluded()));

    // Named Commands
    NamedCommands.registerCommand("lEDTest", lED.solidCommand(Color.kBlanchedAlmond));
    NamedCommands.registerCommand("in_take", stateController.setIntakeMode());
    NamedCommands.registerCommand("intake2", superStructure.goToSource());
    NamedCommands.registerCommand("stopIntake", manipulator.stopIntaking());
    NamedCommands.registerCommand(
        "fire",
        new InstantCommand(() -> manipulator.runWheels(ManipulatorConstants.coralShoot))
            .andThen(Commands.waitSeconds(0.25))
            .andThen(manipulator.stopIntake()));
    NamedCommands.registerCommand(
        "AlgaeFire",
        Commands.runOnce(() -> manipulator.runWheels(ManipulatorConstants.bargeShoot))
            .repeatedly()
            .withTimeout(0.265));
    NamedCommands.registerCommand(
        "goTo_Elevator_State",
        Commands.select(
            Map.ofEntries(
                // L1 Entry
                Map.entry(
                    LevelState.L1, // L1 State
                    // Command at state l1
                    arm.coralL1()
                        .andThen(
                            new ElevatorToSetpoint(elevator, ElevatorConstants.homePos, true)
                                .until(() -> !elevator.mahoming)
                                .andThen(elevator.runOnce(() -> elevator.stop())))),
                // L2 Entry
                Map.entry(
                    LevelState.L2, // L2 State
                    // Command at state l2
                    arm.coralL2()
                        .andThen(
                            new ElevatorToSetpoint(elevator, ElevatorConstants.homePos, true)
                                .until(() -> !elevator.mahoming)
                                .andThen(elevator.runOnce(() -> elevator.stop())))),
                // L3 Entry
                Map.entry(
                    LevelState.L3, // L3 State
                    // Command at sate L3
                    arm.coralL3()
                        .andThen(new ElevatorToSetpoint(elevator, ElevatorConstants.l3Pos))),
                Map.entry(
                    LevelState.L4, // L4 State
                    // Command at state l4
                    new ElevatorToSetpoint(elevator, ElevatorConstants.l4Pos)
                        .alongWith(Commands.waitSeconds(0.5).andThen(arm.coralL4())))),
            stateController::getLevel));
    NamedCommands.registerCommand("SetCoral", stateController.setCoralMode());
    NamedCommands.registerCommand("SetAlgae", stateController.setAlgaeMode());
    NamedCommands.registerCommand("SetL4", stateController.setL4());
    NamedCommands.registerCommand("SetL3", stateController.setL3());
    NamedCommands.registerCommand("SetL1", stateController.setL1());
    NamedCommands.registerCommand("SetL2", stateController.setL2());
    NamedCommands.registerCommand(
        "Home",
        superStructure
            .goHome()
            .alongWith(new ElevatorToSetpoint(elevator, ElevatorConstants.homePos, true))
            .until(() -> !elevator.mahoming)
            .andThen(elevator.runOnce(() -> elevator.stop())));
    NamedCommands.registerCommand(
        "AlgaeHome",
        new ElevatorToSetpoint(elevator, ElevatorConstants.homePos, true, true)
            .until(() -> !elevator.mahoming)
            .andThen(elevator.runOnce(() -> elevator.stop())));

    NamedCommands.registerCommand(
        "waitUntilGamePiece", Commands.waitUntil(manipulator::hasGamePiece));

    NamedCommands.registerCommand(
        "AlgaeL2",
        superStructure
            .goToL2Algae()
            .alongWith(
                new ElevatorToSetpoint(elevator, ElevatorConstants.algaeL2Pos, false, true)));
    NamedCommands.registerCommand(
        "AlgaeL3",
        superStructure
            .goToL3Algae()
            .alongWith(
                new ElevatorToSetpoint(elevator, ElevatorConstants.algaeL3Pos, false, true)));
    NamedCommands.registerCommand(
        "Barge",
        superStructure
            .barge()
            .alongWith(new ElevatorToSetpoint(elevator, ElevatorConstants.bargePos)));
    NamedCommands.registerCommand("Processor", superStructure.goToProcessor());
    NamedCommands.registerCommand("KeepAlgaeIn", manipulator.keepAlgaeIn());
    NamedCommands.registerCommand(
        "waitUntillL4",
        Commands.waitUntil(() -> Math.abs(elevator.getPos() - ElevatorConstants.l4Pos) < 0.50001));
    NamedCommands.registerCommand(
        "waitUntillAL3",
        Commands.waitUntil(
            () -> Math.abs(elevator.getPos() - ElevatorConstants.algaeL3Pos) < 0.25));
    NamedCommands.registerCommand(
        "waitUntillAL2",
        Commands.waitUntil(
            () -> Math.abs(elevator.getPos() - ElevatorConstants.algaeL2Pos) < 0.25));

    configureButtonBindings();

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    // autoChooser.addOption(
    //     "Right3PieceAutoAlineOnly",
    //     new ThreePieceRight(
    //         stateController, elevator, manipulator, arm, drive, this, superStructure));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    /* Triggers */
    Trigger fireReadyAuto =
        new Trigger(() -> stateController.autoReadyFire(arm, elevator, manipulator));

    Trigger rumbleTime = new Trigger(() -> Timer.getMatchTime() <= 20 && Timer.getMatchTime() > 18);

    Trigger coralMode = new Trigger(() -> stateController.isCoralMode());
    Trigger algaeMode = new Trigger(() -> stateController.isAlgaeMode());
    Trigger climbMode = new Trigger(() -> stateController.isClimbMode());
    Trigger intakeMode = new Trigger(() -> stateController.isIntakeMode());
    Trigger goGoGadgetIntake = new Trigger(() -> stateController.isLongIntakeMode());

    Trigger hasGamePiece = new Trigger(() -> stateController.hasGamePiece(manipulator));
    Trigger hasNoGamePiece = new Trigger(() -> !stateController.hasGamePiece(manipulator));
    Trigger gotGamePieceAutoAlgae = new Trigger(() -> stateController.gotGamePieceAutoAlgae());

    Trigger autoAlineModeHathConcluded = new Trigger(() -> stateController.hathConcluded());

    BooleanSupplier slowMode = new Trigger(() -> driverController.getRightTriggerAxis() > 0.01);

    // Rumble controler for 1s when endgame
    rumbleTime.onTrue(
        new InstantCommand(() -> driverController.setRumble(GenericHID.RumbleType.kRightRumble, .8))
            .andThen(new WaitCommand(1))
            .andThen(() -> driverController.setRumble(GenericHID.RumbleType.kRightRumble, 0)));

    // Auto fire
    fireReadyAuto.onTrue(
        new InstantCommand(() -> manipulator.runWheels(ManipulatorConstants.coralShoot))
            .andThen(
                Commands.waitSeconds(0.3604)
                    .andThen(
                        manipulator
                            .stopIntake()
                            .alongWith(
                                superStructure
                                    .goHome()
                                    .andThen(
                                        Commands.runOnce(
                                            () -> stateController.setHathntConcluded()))
                                    .andThen(
                                        new ElevatorToSetpoint(
                                                elevator, ElevatorConstants.homePos, true)
                                            .until(() -> !elevator.mahoming)
                                            .andThen(elevator.runOnce(() -> elevator.stop())))))));

    // auto aline is done pathing, auto raise elevator
    autoAlineModeHathConcluded
        .and(coralMode)
        .onTrue(
            Commands.select(
                Map.ofEntries(
                    // L1 Entry
                    Map.entry(
                        LevelState.L1, // L1 State
                        // Command at state l1
                        arm.coralL1()
                            .andThen(
                                new ElevatorToSetpoint(elevator, ElevatorConstants.homePos, true)
                                    .until(() -> !elevator.mahoming)
                                    .andThen(elevator.runOnce(() -> elevator.stop())))),
                    // L2 Entry
                    Map.entry(
                        LevelState.L2, // L2 State
                        // Command at state l2
                        arm.coralL2()
                            .andThen(
                                new ElevatorToSetpoint(elevator, ElevatorConstants.homePos, true)
                                    .until(() -> !elevator.mahoming)
                                    .andThen(elevator.runOnce(() -> elevator.stop())))),
                    // L3 Entry
                    Map.entry(
                        LevelState.L3, // L3 State
                        // Command at sate L3
                        arm.coralL3()
                            .andThen(new ElevatorToSetpoint(elevator, ElevatorConstants.l3Pos))),
                    Map.entry(
                        LevelState.L4, // L4 State
                        // Command at state l4
                        new ElevatorToSetpoint(elevator, ElevatorConstants.l4Pos)
                            .alongWith(Commands.waitSeconds(0.5).andThen(arm.coralL4())))),
                stateController::getLevel));

    // Auto barge after done pathing
    autoAlineModeHathConcluded
        .and(algaeMode)
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> arm.barge()),
                new ElevatorToSetpoint(elevator, ElevatorConstants.bargePos, false)
                    .until(() -> Math.abs(elevator.getPos() - ElevatorConstants.bargePos) < 0.5),
                Commands.waitSeconds(0.2),
                manipulator.shootAlgaeFaster().repeatedly().withTimeout(0.3),
                manipulator.stopIntaking(),
                new ElevatorToSetpoint(elevator, ElevatorConstants.homePos, true)
                    .until(() -> !elevator.mahoming)
                    .andThen(
                        elevator.runOnce(() -> elevator.stop()),
                        Commands.runOnce(
                            () -> stateController.setHathntConcluded())))); // semicolon by lucas :3

    /* Intake mode */

    // Long intake
    intakeMode.and(goGoGadgetIntake).onTrue(arm.looooongIntake());

    // Intake coral from source
    intakeMode
        .and(coralMode)
        .onTrue(
            superStructure
                .goToSource()
                .alongWith(new ElevatorToSetpoint(elevator, ElevatorConstants.homePos, true))
                .until(() -> !elevator.mahoming)
                .andThen(elevator.runOnce(() -> elevator.stop())));

    intakeMode.and(algaeMode).onTrue(superStructure.intakeFromGround());
    intakeMode.and(algaeMode).and(hasGamePiece).onTrue(stateController.setNoIntakeMode());
    intakeMode.and(algaeMode).onFalse(superStructure.goToProcessor());

    // when get a game piece, set intake mode to false
    intakeMode.and(coralMode).and(hasGamePiece).onTrue(stateController.setNoIntakeMode());

    // intake mode on false, home the robot
    intakeMode
        .and(coralMode)
        .onFalse(
            superStructure
                .goHome()
                .alongWith(Commands.runOnce(() -> stateController.setShortIntake()))
            /*  .andThen(
            new ElevatorToSetpoint(elevator, ElevatorConstants.homePos, true)
                .until(() -> !elevator.mahoming)
                .andThen(elevator.runOnce(() -> elevator.stop())))*/ );

    // Climb mode stops intake wheels
    climbMode.onTrue(manipulator.stopIntake());

    /* DRIVER BUTTONS */

    // Default drive command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            slowMode));

    // Angle towards algae
    // driverController
    //     .y()
    //     .whileTrue(
    //     DriveCommands.angleTowardsAlgae(
    //         drive,
    //         () -> -driverController.getLeftY(),
    //         () -> -driverController.getLeftX(),
    //         () ->
    //             LimelightHelpers.getTV("") ? LimelightHelpers.getTX("") * (Math.PI / 180) : 0));

    // Coral mode + has piece -> angle towards reef panel
    driverController
        .b()
        .and(coralMode)
        .and(hasGamePiece)
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> AllianceFlipUtil.apply(drive.getClosestReefPanel()).getRotation()));

    // Coral mode + no piece -> angle towards source
    driverController
        .b()
        .and(coralMode)
        .and(hasNoGamePiece)
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> AllianceFlipUtil.apply(drive.getClosestSource()).getRotation()));

    // Algae mode + has piece -> angle towards processor
    driverController
        .b()
        .and(algaeMode)
        .and(hasGamePiece)
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> AllianceFlipUtil.apply(FieldConstants.Processor.centerFace).getRotation()));

    // Algae mode + no piece -> angle towards reef
    driverController
        .b()
        .and(algaeMode)
        .and(hasNoGamePiece)
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> AllianceFlipUtil.apply(drive.getClosestReefPanel()).getRotation()));

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

    driverController
        .back()
        .and(coralMode)
        .onTrue(
            Commands.either(
                stateController.setNoIntakeMode(),
                arm.home()
                    .alongWith(
                        new ElevatorToSetpoint(elevator, ElevatorConstants.homePos, true)
                            .until(() -> !elevator.mahoming)
                            .andThen(elevator.runOnce(() -> elevator.stop())))
                    .andThen(stateController.setIntakeMode()),
                intakeMode));

    /* SOURCE PATHFINDS */

    // Left bumper, coral mode, no piece -> left source
    driverController
        .leftBumper()
        .and(coralMode)
        .and(hasNoGamePiece)
        .whileTrue(
            Commands.defer(
                    () -> AutoAline.autoAlineToPose(this, stateController.getSourcePose(true)),
                    Set.of(drive))
                .andThen(lED.strobeCommand(Color.kDarkOrange, .333)));

    // Right bumper, coral mode, no piece -> right source
    driverController
        .rightBumper()
        .and(coralMode)
        .and(hasNoGamePiece)
        .whileTrue(
            Commands.defer(
                    () -> AutoAline.autoAlineToPose(this, stateController.getSourcePose(false)),
                    Set.of(drive))
                .andThen(lED.strobeCommand(Color.kDarkOrange, .333)));

    // L/R Bumper drives to state controller branch (reef box)
    driverController
        .leftBumper()
        .or(driverController.rightBumper())
        .and(coralMode)
        .and(hasGamePiece)
        .whileTrue(
            Commands.defer(
                () ->
                    AutoAline.autoAlineToPose(this, stateController.getBranch())
                        .andThen(Commands.runOnce(() -> stateController.setHathConcluded())),
                Set.of(drive)));

    /* ALGAE PATHFINDS */

    // Left bumper, algae mode, has piece -> barge
    driverController
        .leftBumper()
        .and(algaeMode)
        .and(hasGamePiece)
        .and(gotGamePieceAutoAlgae.negate())
        .whileTrue(
            Commands.sequence(
                Commands.defer(() -> AutoAline.autoAlineToBarge(this), Set.of(drive)),
                Commands.runOnce(() -> stateController.setHathConcluded())));

    // Right bumper, algae mode, has piece -> processor
    driverController
        .rightBumper()
        .and(algaeMode)
        .and(hasGamePiece)
        .and(gotGamePieceAutoAlgae.negate())
        .whileTrue(
            Commands.sequence(
                Commands.defer(() -> AutoAline.autoAlineToProcessorPose(this), Set.of(drive)),
                manipulator.shootAlgae(stateController.isL4())));

    // Right or Left bumper, algae mode, no game piece -> closest reef panel
    driverController
        .rightBumper()
        .or(driverController.leftBumper())
        .and(algaeMode)
        .and(hasNoGamePiece)
        .whileTrue(
            Commands.sequence(
                Commands.runOnce(() -> stateController.setGotPieceAutoAlgae()),
                Commands.defer(
                    () -> AutoAline.autoAlineToAlgaeReefPose(this, superStructure),
                    Set.of(drive))));

    driverController
        .rightBumper()
        .or(driverController.leftBumper())
        .onFalse(Commands.runOnce(() -> stateController.setNotGotPieceAutoAlgae()));

    /* OPERATOR BUTTONS */

    // Set mode to coral
    operatorButtonBox
        .button(2)
        .onTrue(
            stateController
                .setCoralMode()
                .andThen(manipulator.stopIntake())
                .andThen(Commands.runOnce(() -> stateController.setShortIntake()))
                .andThen(Commands.runOnce(() -> stateController.setHathntConcluded())));

    // Set mode to algae
    operatorButtonBox
        .button(1)
        .onTrue(
            stateController
                .setAlgaeMode()
                .andThen(
                    stateController
                        .setNoIntakeMode()
                        .andThen(Commands.runOnce(() -> stateController.setHathntConcluded()))));

    // Set mode to climb when button coral and algae are pressed
    operatorButtonBox
        .button(1)
        .and(operatorButtonBox.button(2))
        .onTrue(stateController.setClimbMode());

    // Set L4 Coral
    operatorButtonBox.button(3).and(coralMode).onTrue(stateController.setL4());

    // Go to barge positions
    operatorButtonBox
        .button(3)
        .and(algaeMode)
        .onTrue(
            stateController
                .setL4()
                .alongWith(
                    superStructure
                        .barge()
                        .alongWith(new ElevatorToSetpoint(elevator, ElevatorConstants.bargePos))));

    // Set L3 Coral
    operatorButtonBox.button(4).and(coralMode).onTrue(stateController.setL3());

    // Intake L3 algae
    operatorButtonBox
        .button(4)
        .and(algaeMode)
        .onTrue(
            stateController
                .setNoIntakeMode()
                .andThen(
                    stateController
                        .setL3()
                        .alongWith(
                            superStructure
                                .goToL3Algae()
                                .alongWith(
                                    new ElevatorToSetpoint(elevator, ElevatorConstants.algaeL3Pos))
                                .until(hasGamePiece)
                                .andThen(manipulator.keepAlgaeIn()))));

    // Set L2 Coral
    operatorButtonBox.button(5).and(coralMode).onTrue(stateController.setL2());

    // Intake L2 algae
    operatorButtonBox
        .button(5)
        .and(algaeMode)
        .onTrue(
            stateController
                .setNoIntakeMode()
                .andThen(
                    stateController
                        .setL2()
                        .alongWith(
                            superStructure
                                .goToL2Algae()
                                .alongWith(
                                    new ElevatorToSetpoint(elevator, ElevatorConstants.algaeL2Pos))
                                .until(hasGamePiece)
                                .andThen(manipulator.keepAlgaeIn()))));

    // Set L1 Coral
    operatorButtonBox
        .button(6)
        .and(coralMode)
        .onTrue(
            arm.coralL1()
                .alongWith(stateController.setL1())
                .alongWith(new ElevatorToSetpoint(elevator, ElevatorConstants.l1Pos)));

    // If game piece -> Processor position
    operatorButtonBox
        .button(6)
        .and(algaeMode)
        .and(hasGamePiece)
        .onTrue(
            superStructure
                .goToProcessor()
                .alongWith(
                    new ElevatorToSetpoint(elevator, ElevatorConstants.homePos, true, true)
                        .until(() -> !elevator.mahoming)
                        .andThen(elevator.runOnce(() -> elevator.stop()))));

    // If no game piece -> lolipop intake position
    operatorButtonBox
        .button(6)
        .and(algaeMode)
        .and(hasNoGamePiece)
        .onTrue(
            new ElevatorToSetpoint(elevator, ElevatorConstants.homePos, true, true)
                .until(() -> !elevator.mahoming)
                .andThen(elevator.runOnce(() -> elevator.stop()))
                .alongWith(
                    superStructure
                        .goToLolliPop()
                        .repeatedly()
                        .until(hasGamePiece)
                        .andThen(superStructure.goToProcessor())));

    // Goes home (i love patrick mahomes)
    operatorButtonBox
        .button(7)
        .and(coralMode)
        .onTrue(
            arm.home()
                .alongWith(
                    stateController
                        .setNoIntakeMode()
                        .andThen(manipulator.stopIntake())
                        .alongWith(
                            new ElevatorToSetpoint(elevator, ElevatorConstants.homePos, true)))
                .until(() -> !elevator.mahoming)
                .andThen(elevator.runOnce(() -> elevator.stop())));

    // Processor position
    operatorButtonBox
        .button(7)
        .and(algaeMode)
        .onTrue(
            stateController
                .setMahome()
                .alongWith(
                    superStructure
                        .goToProcessor()
                        .alongWith(
                            new ElevatorToSetpoint(elevator, ElevatorConstants.homePos, true, true))
                        .until(() -> !elevator.mahoming)
                        .andThen(elevator.runOnce(() -> elevator.stop()))));

    // Set arm to climb position
    operatorButtonBox.button(8).and(climbMode).onTrue(arm.climb());

    // gogogadget INTAKE!!
    operatorButtonBox
        .button(9)
        .and(intakeMode)
        .onTrue(Commands.runOnce(() -> stateController.setLongIntake()));

    operatorButtonBox
        .button(10)
        .onTrue(
            Commands.either(
                stateController.setNoIntakeMode(), stateController.setIntakeMode(), intakeMode));

    // Manual elevator
    operatorButtonBox
        .button(11)
        .onTrue(
            Commands.select(
                Map.ofEntries(
                    // L1 Entry
                    Map.entry(
                        LevelState.L1, // L1 State
                        // Command at state l1
                        arm.coralL1()
                            .andThen(
                                new ElevatorToSetpoint(elevator, ElevatorConstants.homePos, true)
                                    .until(() -> !elevator.mahoming)
                                    .andThen(elevator.runOnce(() -> elevator.stop())))),
                    // L2 Entry
                    Map.entry(
                        LevelState.L2, // L2 State
                        // Command at state l2
                        arm.coralL2()
                            .andThen(
                                new ElevatorToSetpoint(elevator, ElevatorConstants.homePos, true)
                                    .until(() -> !elevator.mahoming)
                                    .andThen(elevator.runOnce(() -> elevator.stop())))
                            .until(() -> !stateController.isL2())),
                    // L3 Entry
                    Map.entry(
                        LevelState.L3, // L3 State
                        // Command at sate L3
                        arm.coralL3()
                            .andThen(new ElevatorToSetpoint(elevator, ElevatorConstants.l3Pos))
                            .until(() -> !stateController.isL3())),
                    Map.entry(
                        LevelState.L4, // L4 State
                        // Command at state l4
                        new ElevatorToSetpoint(elevator, ElevatorConstants.l4Pos)
                            .alongWith(Commands.waitSeconds(0.5).andThen(arm.coralL4()))
                            .until(() -> !stateController.isL4()))),
                stateController::getLevel));

    // Fire coral manually
    operatorButtonBox
        .button(12)
        .and(coralMode)
        .onTrue(
            manipulator
                .shootCoral()
                .alongWith(Commands.runOnce(() -> stateController.setHathntConcluded())));

    // fire button on false -> home robot
    operatorButtonBox
        .button(12)
        .and(coralMode)
        .onFalse(
            manipulator
                .stopIntake()
                .alongWith(arm.home())
                .alongWith(new ElevatorToSetpoint(elevator, ElevatorConstants.homePos, true))
                .until(() -> !elevator.mahoming)
                .andThen(elevator.runOnce(() -> elevator.stop())));

    // fire algae manually, power dependant on L4
    operatorButtonBox
        .button(12)
        .and(algaeMode)
        .onTrue(
            Commands.either(
                manipulator.shootAlgaeFaster(),
                manipulator.shootAlgaeSlower(),
                stateController::isL4));

    // fire button on false -> home elevator
    operatorButtonBox
        .button(12)
        .and(algaeMode)
        .onFalse(
            manipulator
                .stopIntake()
                .alongWith(new ElevatorToSetpoint(elevator, ElevatorConstants.homePos, true))
                .until(() -> !elevator.mahoming)
                .andThen(elevator.runOnce(() -> elevator.stop())));

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
    operatorReefBox.button(1).onTrue(stateController.setBranch(Branch.FRONTRIGHT_RIGHTBRANCH));
    operatorReefBox.button(2).onTrue(stateController.setBranch(Branch.FRONTRIGHT_LEFTBRANCH));
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

// "the code is now very cleam" - andrew 3/10/2025

// andrew is super awesome and cool

// so is lucas

// not simon and gavin tho (haha)

// line 1k so cool

// its not line 1k anymore, haha
