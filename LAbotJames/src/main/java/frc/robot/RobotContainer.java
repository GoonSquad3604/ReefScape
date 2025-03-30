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
  private final Climber climber;
  public final Elevator elevator;
  private final LEDs lED;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandJoystick operatorButtonBox = new CommandJoystick(1);
  private final CommandJoystick operatorReefBox = new CommandJoystick(2);
  private final CommandXboxController testController = new CommandXboxController(3);
  private final Supplier<Translation2d> joystickSupplier =
      () -> new Translation2d(driverController.getLeftY(), driverController.getLeftX());

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Commands
  //   private Command goHome;

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
    superStructure = new SuperStructure(manipulator, arm, elevator, stateController);

    lED.setDefaultCommand(
        lED.defaultLeds(() -> stateController.getMode(), () -> stateController.isIntakeMode()));

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
        Commands.runOnce(() -> manipulator.runWheels(ManipulatorConstants.bargeShoot)));
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
                    arm.coralL4()
                        .andThen(new ElevatorToSetpoint(elevator, ElevatorConstants.l4Pos)))),
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
        new ElevatorToSetpoint(elevator, ElevatorConstants.homePos, true)
            .until(() -> !elevator.mahoming)
            .andThen(elevator.runOnce(() -> elevator.stop())));

    NamedCommands.registerCommand(
        "waitUntilGamePiece", Commands.waitUntil(manipulator::hasGamePiece));

    NamedCommands.registerCommand(
        "AlgaeL2",
        superStructure
            .goToL2Algae()
            .alongWith(new ElevatorToSetpoint(elevator, ElevatorConstants.algaeL2Pos)));
    NamedCommands.registerCommand(
        "AlgaeL3",
        superStructure
            .goToL3Algae()
            .alongWith(new ElevatorToSetpoint(elevator, ElevatorConstants.algaeL3Pos)));
    NamedCommands.registerCommand(
        "Barge",
        superStructure
            .barge()
            .alongWith(new ElevatorToSetpoint(elevator, ElevatorConstants.bargePos)));
    NamedCommands.registerCommand("Processor", superStructure.goToProcessor());
    NamedCommands.registerCommand("KeepAlgaeIn", manipulator.keepAlgaeIn());

    configureButtonBindings();

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    //  autoChooser.addOption(
    //      "Drive Wheel Radius Characterization",
    // DriveCommands.wheelRadiusCharacterization(drive));
    //  autoChooser.addOption(
    //      "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    //  autoChooser.addOption(
    //      "Drive SysId (Quasistatic Forward)",
    //      drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    //  autoChooser.addOption(
    //      "Drive SysId (Quasistatic Reverse)",
    //      drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    //  autoChooser.addOption(
    //      "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    //  autoChooser.addOption(
    //      "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

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

    Trigger L4 = new Trigger(() -> stateController.isL4());
    Trigger L3 = new Trigger(() -> stateController.isL3());
    Trigger L2 = new Trigger(() -> stateController.isL2());
    Trigger L1 = new Trigger(() -> stateController.isL1());
    Trigger LMahome = new Trigger(() -> stateController.isMahome());

    // Trigger operatorManualOverride = operatorButtonBox.button(11);
    // Trigger driverPathOverride = new Trigger(() -> driverController.getLeftTriggerAxis() > 0.01);
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
                                        new ElevatorToSetpoint(
                                                elevator, ElevatorConstants.homePos, true)
                                            .until(() -> !elevator.mahoming)
                                            .andThen(elevator.runOnce(() -> elevator.stop())))))));

    // Default drive command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            slowMode));

    /* DRIVER BUTTONS */

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

    driverController
        .povDown()
        // .or(driverController.povDownLeft())
        // .or(driverController.povDownRight())
        .and(climbMode)
        .onTrue(climber.moveClimberDown());
    driverController
        .povDown()
        // .or(driverController.povDownLeft())
        // .or(driverController.povDownRight())
        .and(climbMode)
        .onFalse(climber.stop());

    driverController
        .back()
        .and(coralMode)
        .onTrue(
            arm.home()
                .alongWith(
                    new ElevatorToSetpoint(elevator, ElevatorConstants.homePos, true)
                        .until(() -> !elevator.mahoming)
                        .andThen(elevator.runOnce(() -> elevator.stop())))
                .andThen(stateController.setIntakeMode()));

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

    /* MANUAL CORAL PATHFINDS */

    // driverController
    //     .povRight()
    //     .onTrue(
    //         Commands.select(
    //             Map.ofEntries(
    //                 // L1 Entry
    //                 Map.entry(
    //                     LevelState.L1, // L1 State
    //                     // Command at state l1
    //                     arm.coralL1()
    //                         .andThen(
    //                             new ElevatorToSetpoint(elevator, ElevatorConstants.homePos, true)
    //                                 .until(() -> !elevator.mahoming)
    //                                 .andThen(elevator.runOnce(() -> elevator.stop())))),
    //                 // L2 Entry
    //                 Map.entry(
    //                     LevelState.L2, // L2 State
    //                     // Command at state l2
    //                     arm.coralL2()
    //                         .andThen(
    //                             new ElevatorToSetpoint(elevator, ElevatorConstants.homePos, true)
    //                                 .until(() -> !elevator.mahoming)
    //                                 .andThen(elevator.runOnce(() -> elevator.stop())))),
    //                 // L3 Entry
    //                 Map.entry(
    //                     LevelState.L3, // L3 State
    //                     // Command at sate L3
    //                     arm.coralL3()
    //                         .andThen(new ElevatorToSetpoint(elevator, ElevatorConstants.l3Pos))),
    //                 Map.entry(
    //                     LevelState.L4, // L4 State
    //                     // Command at state l4
    //                     arm.coralL4()
    //                         .andThen(new ElevatorToSetpoint(elevator,
    // ElevatorConstants.l4Pos)))),
    //             stateController::getLevel));

    // OVERRIDE, Left bumper, coral mode, has piece -> closest left pole
    // driverController
    //     .leftBumper()
    //     .and(coralMode)
    //     .and(hasGamePiece)
    //     .and(driverPathOverride)
    //     .whileTrue(
    //         Commands.defer(
    //                 () -> AutoAline.autoAlineTo(Target.LEFT_POLE, this, joystickSupplier),
    //                 Set.of(drive))
    //             .andThen(lED.strobeCommand(Color.kDarkOrange, .333)));

    // OVERRIDE, Right bumper, coral mode, has piece -> closest right pole
    // driverController
    //     .rightBumper()
    //     .and(coralMode)
    //     .and(hasGamePiece)
    //     .and(driverPathOverride)
    //     .whileTrue(
    //         Commands.defer(
    //                 () -> AutoAline.autoAlineTo(Target.RIGHT_POLE, this, joystickSupplier),
    //                 Set.of(drive))
    //             .andThen(lED.strobeCommand(Color.kDarkOrange, .333)));

    /* SOURCE PATHFINDS */

    // Left bumper, coral mode, no piece -> left source
    driverController
        .leftBumper()
        .and(coralMode)
        .and(hasNoGamePiece)
        .whileTrue(
            Commands.defer(
                    () ->
                        AutoAline.autoAlineToPose(
                            this, FieldConstants.CoralStation.leftCenterIntakePos),
                    Set.of(drive))
                .andThen(lED.strobeCommand(Color.kDarkOrange, .333)));

    // Right bumper, coral mode, no piece -> right source
    driverController
        .rightBumper()
        .and(coralMode)
        .and(hasNoGamePiece)
        .whileTrue(
            Commands.defer(
                    () ->
                        AutoAline.autoAlineToPose(
                            this, FieldConstants.CoralStation.rightCenterIntakePos),
                    Set.of(drive))
                .andThen(lED.strobeCommand(Color.kDarkOrange, .333)));

    /* DEFAULT CORAL PATHFINDS */
    driverController
        .rightBumper()
        .and(coralMode)
        .and(hasGamePiece)
        // .and(driverPathOverride.negate())
        .whileTrue(
            Commands.defer(
                () ->
                    AutoAline.autoAlineToPose(this, stateController.getBranch())
                        .andThen(lED.strobeCommand(Color.kDarkOrange, .333)),
                Set.of(drive)));

    driverController
        .leftBumper()
        .and(coralMode)
        .and(hasGamePiece)
        // .and(driverPathOverride.negate())
        .whileTrue(
            Commands.defer(
                () ->
                    AutoAline.autoAlineToPose(this, stateController.getBranch())
                        .andThen(lED.strobeCommand(Color.kDarkOrange, .333)),
                Set.of(drive)));

    /* ALGAE PATHFINDS */

    // Left bumper, algae mode, has piece -> barge (probably wont want, idk)
    // driverController
    //     .leftBumper()
    //     .and(algaeMode)
    //     .and(hasGamePiece)
    //     .whileTrue(
    //         drive.defer(
    //             () ->
    //                 drive
    //                     .pathfindToFieldPose(
    //                         AllianceFlipUtil.apply(FieldConstants.Processor.centerFace))
    //                     .andThen(lED.strobeCommand(Color.kDarkOrange, .333))));

    // Right bumper, algae mode, has piece -> processor
    driverController
        .rightBumper()
        .and(algaeMode)
        .and(hasGamePiece)
        .whileTrue(
            Commands.defer(
                    () -> drive.pathfindToPath(stateController.getAlgaePath(true)), Set.of(drive))
                .andThen(lED.strobeCommand(Color.kDarkOrange, .333)));

    // Left bumper, algae mode, no piece -> closest reef center face (same as right bumper)
    // doesnt work rn
    // driverController
    //     .leftBumper()
    //     .and(algaeMode)
    //     .and(hasNoGamePiece)
    //     .whileTrue(
    //         Commands.defer(
    //             () ->
    //                 drive
    //                     .pathfindToPath(drive.getClosestReefPath())
    //                     .andThen(lED.strobeCommand(Color.kDarkOrange, .333)),
    //             Set.of(drive)));

    // Right bumper, algae mode, no piece -> closest reef center face (same as left bumper)
    // doesnt work rn
    // driverController
    //     .rightBumper()
    //     .and(algaeMode)
    //     .and(hasNoGamePiece)
    //     .whileTrue(
    //         Commands.defer(
    //             () ->
    //                 drive
    //                     .pathfindToPath(drive.getClosestReefPath())
    //                     .andThen(lED.strobeCommand(Color.kDarkOrange, .333)),
    //             Set.of(drive)));

    /* CLIMB PATHFIND */

    driverController
        .leftBumper()
        .or(driverController.rightBumper())
        .and(climbMode)
        .whileTrue(
            Commands.defer(
                () ->
                    new ElevatorToSetpoint(elevator, ElevatorConstants.homePos, true)
                        .until(() -> !elevator.mahoming)
                        .andThen(elevator.runOnce(() -> elevator.stop()))
                        .alongWith(
                            arm.climb()
                                .andThen(
                                    drive
                                        .pathfindToPath(climber.getClimbPath())
                                        .andThen(climber.setClimberUp()))),
                Set.of(drive, climber)));

    /* OPERATOR BUTTONS */

    // Set mode to coral
    operatorButtonBox
        .button(1 + 1)
        .onTrue(
            stateController
                .setCoralMode()
                .andThen(manipulator.stopIntake())
                .andThen(Commands.runOnce(() -> stateController.setShortIntake())));

    // Set mode to algae
    operatorButtonBox
        .button(2 - 1)
        .onTrue(stateController.setAlgaeMode().andThen(stateController.setNoIntakeMode()));

    // Set mode to climb when button coral and algae are pressed
    operatorButtonBox
        .button(1)
        .and(operatorButtonBox.button(2))
        .onTrue(stateController.setClimbMode(manipulator));

    // L4 Coral (queue)
    // operatorButtonBox
    //     .button(3)
    //     .and(coralMode)
    //     .and(operatorManualOverride.negate())
    //     .onTrue(stateController.setL4());

    // L4 Coral (manual)
    operatorButtonBox
        .button(3)
        .and(coralMode)
        .onTrue(
            stateController
                .setL4()
                .alongWith(
                    new ElevatorToSetpoint(elevator, ElevatorConstants.l4Pos)
                        .alongWith(Commands.waitSeconds(0.5).andThen(arm.coralL4()))));

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

    // L3 Coral (queue)
    // operatorButtonBox
    //     .button(4)
    //     .and(coralMode)
    //     .and(operatorManualOverride.negate())
    //     .onTrue(stateController.setL3());

    // L3 Coral (manual)
    operatorButtonBox
        .button(4)
        .and(coralMode)
        // .and(operatorManualOverride)
        .onTrue(
            arm.coralL3()
                .alongWith(
                    new ElevatorToSetpoint(elevator, ElevatorConstants.l3Pos)
                        .alongWith(stateController.setL3())));

    // Intake L3 algae
    operatorButtonBox
        .button(4)
        .and(algaeMode)
        .onTrue(
            stateController
                .setL3()
                .alongWith(
                    superStructure
                        .goToL3Algae()
                        .alongWith(new ElevatorToSetpoint(elevator, ElevatorConstants.algaeL3Pos))
                        .until(hasGamePiece)
                        .andThen(manipulator.keepAlgaeIn())));

    // // Return elevator to home on release
    // operatorButtonBox
    //     .button(4)
    //     .and(algaeMode)
    //     .onFalse(
    //         stateController.setMahome().alongWith(
    //         superStructure
    //             .goToProcessor()
    //             .alongWith(
    //                 new ElevatorToSetpoint(elevator, ElevatorConstants.homePos, true)
    //                     .until(() -> !elevator.mahoming)
    //                     .andThen(elevator.runOnce(() -> elevator.stop())))));

    // L2 Coral (queue)
    // operatorButtonBox
    //     .button(5)
    //     .and(coralMode)
    //     .and(operatorManualOverride.negate())
    //     .onTrue(stateController.setL2());

    // L2 Coral (manual)
    operatorButtonBox
        .button(5)
        .and(coralMode)
        .onTrue(
            arm.coralL2()
                .alongWith(stateController.setL2())
                .alongWith(
                    new ElevatorToSetpoint(elevator, ElevatorConstants.homePos, true)
                        .until(() -> !elevator.mahoming)
                        .andThen(elevator.runOnce(() -> elevator.stop()))));

    // Intake L2 algae
    operatorButtonBox
        .button(5)
        .and(algaeMode)
        .onTrue(
            stateController
                .setL2()
                .alongWith(
                    superStructure
                        .goToL2Algae()
                        .alongWith(new ElevatorToSetpoint(elevator, ElevatorConstants.algaeL2Pos))
                        .until(hasGamePiece)
                        .andThen(manipulator.keepAlgaeIn())));

    // // Return elevator to home on release
    // operatorButtonBox
    //     .button(5)
    //     .and(algaeMode)
    //     .onFalse(
    //         stateController.setMahome().alongWith(
    //         superStructure
    //             .goToProcessor()
    //             .alongWith(
    //                 new ElevatorToSetpoint(elevator, ElevatorConstants.homePos, true)
    //                     .until(() -> !elevator.mahoming)
    //                     .andThen(elevator.runOnce(() -> elevator.stop())))));

    // L1 Coral (queue)
    // operatorButtonBox
    //     .button(6)
    //     .and(coralMode)
    //     .and(operatorManualOverride.negate())
    //     .onTrue(stateController.setL1());

    // L1 Coral (manual)
    operatorButtonBox
        .button(6)
        .and(coralMode)
        .onTrue(
            arm.coralL1()
                .alongWith(stateController.setL1())
                .alongWith(new ElevatorToSetpoint(elevator, ElevatorConstants.l1Pos)));

    operatorButtonBox
        .button(6)
        .and(algaeMode)
        .and(hasGamePiece)
        .onTrue(
            superStructure
                .goToProcessor()
                .alongWith(
                    new ElevatorToSetpoint(elevator, ElevatorConstants.homePos, true)
                        .until(() -> !elevator.mahoming)
                        .andThen(elevator.runOnce(() -> elevator.stop()))));
    operatorButtonBox
        .button(6)
        .and(algaeMode)
        .and(hasNoGamePiece)
        .onTrue(
            new ElevatorToSetpoint(elevator, ElevatorConstants.homePos, true)
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
            stateController
                .setMahome()
                .andThen(
                    arm.home()
                        .alongWith(
                            stateController
                                .setNoIntakeMode()
                                .andThen(manipulator.stopIntake())
                                .alongWith(
                                    new ElevatorToSetpoint(
                                        elevator, ElevatorConstants.homePos, true)))
                        .until(() -> !elevator.mahoming)
                        .andThen(elevator.runOnce(() -> elevator.stop()))));

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
                            new ElevatorToSetpoint(elevator, ElevatorConstants.homePos, true))
                        .until(() -> !elevator.mahoming)
                        .andThen(elevator.runOnce(() -> elevator.stop()))));

    // Set arm to climb position
    operatorButtonBox
        .button(8)
        .and(climbMode)
        .onTrue(arm.climb()); // .alongWith(climber.setClimberHome()));

    // Climb up
    // operatorButtonBox.button(9).and(climbMode).onTrue(climber.setClimberUp());

    // gogogadget INTAKE!!
    operatorButtonBox
        .button(9)
        .and(intakeMode)
        .onTrue(Commands.runOnce(() -> stateController.setLongIntake()));

    intakeMode.and(goGoGadgetIntake).onTrue(arm.looooongIntake());

    // Intake
    // operatorButtonBox
    //     .button(10)
    //     .and(coralMode)
    //     .whileTrue(
    //         superStructure
    //             .goToSource()
    //             .alongWith(
    //                 lED.strobeCommand(Color.kRed, 0.333)
    //                     .until(() -> manipulator.hasGamePiece())
    //                     .andThen(lED.solidCommand(Color.kGreen))));

    // Stop intake - begin manipulator power to hold coral
    // operatorButtonBox
    //     .button(10)
    //     .and(coralMode)
    //     .onFalse(
    //         manipulator
    //             .stopIntake()
    //             .alongWith(arm.home())
    //             .alongWith(lED.defaultLeds(() -> stateController.getMode())));

    // makes the mode = intake
    // tap again, or press home for no intake mode
    // operatorButtonBox.button(10).and(coralMode).onTrue(stateController.setIntakeMode());

    operatorButtonBox
        .button(10)
        // .and(coralMode)
        .onTrue(
            Commands.either(
                stateController.setNoIntakeMode(), stateController.setIntakeMode(), intakeMode));

    // when intake + coral, begin to intake
    intakeMode
        .and(coralMode)
        .onTrue(superStructure.goToSource().alongWith(new ElevatorToSetpoint(elevator, 1.5)));

    intakeMode.and(algaeMode).onTrue(superStructure.intakeFromGround());
    intakeMode.and(algaeMode).onFalse(superStructure.goToProcessor());
    intakeMode.and(algaeMode).and(hasGamePiece).onTrue(stateController.setNoIntakeMode());

    // .until(() -> manipulator.hasGamePiece()))
    // .andThen(
    //     manipulator
    //         .intakeCoral()
    //         .withTimeout(.5)
    //         .andThen(stateController.setNoIntakeMode())));

    intakeMode
        .and(coralMode)
        .onFalse(
            arm.home()
                .alongWith(manipulator.intakeCoral().withTimeout(.5))
                .andThen(manipulator.stopIntake())
                .alongWith(
                    superStructure
                        .goHome()
                        .alongWith(Commands.runOnce(() -> stateController.setShortIntake())))
                .andThen(
                    new ElevatorToSetpoint(elevator, ElevatorConstants.homePos, true)
                        .until(() -> !elevator.mahoming)
                        .andThen(elevator.runOnce(() -> elevator.stop()))));
    // .alongWith(lED.defaultLeds(() -> stateController.getMode()))));

    intakeMode.and(coralMode).and(hasGamePiece).onTrue(stateController.setNoIntakeMode());

    // algaeMode.and(hasGamePiece).whileTrue(manipulator.keepAlgaeIn());
    // algaeMode.and(hasGamePiece.negate()).whileTrue(manipulator.intakeAlgae());

    climbMode.onTrue(manipulator.stopIntake());

    // operatorButtonBox
    //     .button(10)
    //     .and(algaeMode)
    //     .toggleOnTrue(
    //         superStructure
    //             .intakeFromGround()
    //             .repeatedly()
    //             .until(hasGamePiece)
    //             .andThen(superStructure.goToProcessor()));
    /*  .alongWith(
    //     new ElevatorToSetpoint(elevator, ElevatorConstants.homePos, true)
    //         .until(() -> !elevator.mahoming)
        .andThen(elevator.runOnce(() -> elevator.stop())) */

    // operatorButtonBox.button(10).and(algaeMode).toggleOnFalse(superStructure.goToProcessor());

    // Vomit
    operatorButtonBox
        .button(11)
        .whileTrue(manipulator.vomit().alongWith(new ElevatorToSetpoint(elevator, 7.3604)));

    operatorButtonBox
        .button(11)
        .onFalse(elevator.runOnce(() -> elevator.stop()).alongWith(manipulator.stopIntake()));

    // Fire

    operatorButtonBox.button(12).and(coralMode).onTrue(manipulator.shootCoral());

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

    operatorButtonBox.button(12).and(algaeMode).onTrue(manipulator.shootAlgaeFaster());

    operatorButtonBox
        .button(12)
        .and(algaeMode)
        .onFalse(
            manipulator
                .stopIntake()
                .alongWith(new ElevatorToSetpoint(elevator, ElevatorConstants.homePos, true))
                .until(() -> !elevator.mahoming)
                .andThen(elevator.runOnce(() -> elevator.stop())));

    // operatorButtonBox.button(12).and(L1).onTrue(superStructure.fire());
    // operatorButtonBox.button(12).and(L2).onTrue(superStructure.fire());
    // operatorButtonBox.button(12).and(L3).onTrue(superStructure.fire());

    // operatorButtonBox
    //     .button(12)
    //     .and(L4)
    //     .onTrue(
    //         superStructure.fire().until(() -> !manipulator.hasGamePiece()).andThen(arm.layup()));

    // operatorButtonBox
    //     .button(12)
    //     .onFalse(
    //         superStructure
    //             .goHome()
    //             .andThen(
    //                 new ElevatorToSetpoint(elevator, ElevatorConstants.homePos, true)
    //                     .until(() -> !elevator.mahoming)
    //                     .andThen(elevator.runOnce(() -> elevator.stop()))
    //                     .alongWith(stateController.setMahome())));

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

    /* TEST CONTROLLER */

    // testController
    //     .povDown()
    //     .onTrue(
    //         superStructure
    //             .goToSource()
    //             .repeatedly()
    //             .until(hasGamePiece)
    //             .andThen(manipulator.stopIntaking()));
    testController.povLeft().onTrue(arm.elbowUp());
    testController.povLeft().onFalse(arm.stopElbow());

    testController.povRight().onTrue(arm.elbowDown());
    testController.povRight().onFalse(arm.stopElbow());

    testController.y().whileTrue(arm.wristUp());
    testController.y().onFalse(arm.stopWrist());

    testController.a().whileTrue(arm.wristDown());
    testController.a().onFalse(arm.stopWrist());

    testController.start().onTrue(new ElevatorToSetpoint(elevator, 1.5));

    testController
        .back()
        .onTrue(Commands.defer(() -> new ElevatorToSetpoint(elevator, 2), Set.of(elevator)));
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
