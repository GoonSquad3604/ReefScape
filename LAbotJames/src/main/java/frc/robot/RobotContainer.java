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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.AutoAline.Target;
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
  private final StateController stateController;
  private final SuperStructure superStructure;
  private final Arm arm;
  private final Manipulator manipulator;
  private final Climber climber;
  private final Elevator elevator;
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
                // new VisionIOPhotonVision(camera4Name, robotToCamera4),
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

    lED.setDefaultCommand(lED.defaultLeds(() -> stateController.getMode()));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // usable commands

    // autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Named Commands
    NamedCommands.registerCommand("lEDTest", lED.solidCommand(Color.kBlanchedAlmond));
    NamedCommands.registerCommand(
        "intake", superStructure.goToSource().andThen(superStructure.intake()));
    NamedCommands.registerCommand(
        "intakeUntilPiece",
        superStructure
            .goToSource()
            .alongWith(
                lED.strobeCommand(Color.kRed, 0.32)
                    .until(() -> manipulator.hasGamePiece())
                    .andThen(lED.solidCommand(Color.kGreen))));
    NamedCommands.registerCommand(
        "fire", superStructure.fire().withTimeout(2).andThen(superStructure.intakeOff()));
    NamedCommands.registerCommand("holdFire", superStructure.intakeOff());
    NamedCommands.registerCommand(
        "goToL4",
        superStructure
            .goToL4Coral()
            .andThen(stateController.setL4())
            .andThen(new ElevatorToSetpoint(elevator, ElevatorConstants.l4Pos)));
    NamedCommands.registerCommand(
        "fireL4",
        superStructure
            .fire()
            .until(() -> !manipulator.hasGamePiece())
            .andThen(superStructure.layup()));
    NamedCommands.registerCommand("goToAlgae25", superStructure.removeL2Algae());
    NamedCommands.registerCommand("goToAlgae35", superStructure.removeL3Algae());
    NamedCommands.registerCommand("goToSource", superStructure.goToSource());
    NamedCommands.registerCommand("goToProcessor", superStructure.goToProcessor());
    NamedCommands.registerCommand("goToBarge", superStructure.goToBarge());
    NamedCommands.registerCommand(
        "goHome",
        new ElevatorToSetpoint(elevator, ElevatorConstants.homePos, true)
            .until(() -> !elevator.mahoming)
            .andThen(elevator.runOnce(() -> elevator.stop()))
            .alongWith(superStructure.goHome().alongWith(stateController.setMahome())));
    configureButtonBindings();
    NamedCommands.registerCommand(
        "algaeRemoval",
        new ElevatorToSetpoint(elevator, ElevatorConstants.l3PosAlgae)
            .alongWith(superStructure.removeL3Algae().alongWith(stateController.setL3())));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    Trigger rumbleTime = new Trigger(() -> Timer.getMatchTime() <= 20 && Timer.getMatchTime() > 18);
    Trigger coralMode = new Trigger(() -> stateController.isCoralMode());
    Trigger algaeMode = new Trigger(() -> stateController.isAlgaeMode());
    Trigger climbMode = new Trigger(() -> stateController.isClimbMode());
    Trigger hasGamePiece = new Trigger(() -> stateController.hasGamePiece(manipulator));
    Trigger hasNoGamePiece = new Trigger(() -> !stateController.hasGamePiece(manipulator));
    Trigger L4 = new Trigger(() -> stateController.isL4());
    Trigger L3 = new Trigger(() -> stateController.isL3());
    Trigger L2 = new Trigger(() -> stateController.isL2());
    Trigger L1 = new Trigger(() -> stateController.isL1());
    Trigger LMahome = new Trigger(() -> stateController.isMahome());
    BooleanSupplier slowMode = new Trigger(() -> driverController.getRightTriggerAxis() > 0.01);
    // Trigger fireReadyAuto = new Trigger(() -> stateController.autoReadyFire());
    Trigger intakeMode = new Trigger(() -> stateController.isIntakeMode());
    BooleanSupplier manualOverride = new Trigger(() -> operatorButtonBox.button(11).getAsBoolean());
    BooleanSupplier driverPathOverride =
        new Trigger(() -> driverController.getLeftTriggerAxis() > 0.01);

    // Rumble controler for 1s when endgame
    rumbleTime.onTrue(
        new InstantCommand(() -> driverController.setRumble(GenericHID.RumbleType.kRightRumble, .8))
            .andThen(new WaitCommand(1))
            .andThen(() -> driverController.setRumble(GenericHID.RumbleType.kRightRumble, 0)));

    // Auto fire
    // fireReadyAuto.onTrue(new InstantCommand(() -> superStructure.fire()));

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

    /* CORAL PATHFINDS */

    // Left bumper, coral mode, has piece -> closest left pole
    driverController
        .leftBumper()
        .and(coralMode)
        .and(hasGamePiece)
        .and(driverPathOverride)
        .whileTrue(
            Commands.defer(
                    () -> AutoAline.autoAlineTo(Target.LEFT_POLE, this, joystickSupplier),
                    Set.of(drive))
                .andThen(lED.strobeCommand(Color.kDarkOrange, .333)));

    // Right bumper, coral mode, has piece -> closest right pole
    driverController
        .rightBumper()
        .and(coralMode)
        .and(hasGamePiece)
        .and(driverPathOverride)
        .whileTrue(
            Commands.defer(
                    () -> AutoAline.autoAlineTo(Target.RIGHT_POLE, this, joystickSupplier),
                    Set.of(drive))
                .andThen(lED.strobeCommand(Color.kDarkOrange, .333)));

    // Left bumper, coral mode, no piece -> left source
    driverController
        .leftBumper()
        .and(coralMode)
        .and(hasNoGamePiece)
        .whileTrue(
            Commands.defer(
                    () ->
                        drive.pathfindToFieldPose(
                            () ->
                                AllianceFlipUtil.apply(
                                    FieldConstants.CoralStation.leftCenterIntakePos)),
                    Set.of(drive))
                .alongWith(stateController.setIntakeMode())
                .andThen(lED.strobeCommand(Color.kDarkOrange, .333)));

    // Right bumper, coral mode, no piece -> right source
    driverController
        .rightBumper()
        .and(coralMode)
        .and(hasNoGamePiece)
        .whileTrue(
            Commands.defer(
                    () ->
                        drive.pathfindToFieldPose(
                            () ->
                                AllianceFlipUtil.apply(
                                    FieldConstants.CoralStation.rightCenterIntakePos)),
                    Set.of(drive))
                .alongWith(stateController.setIntakeMode())
                .andThen(lED.strobeCommand(Color.kDarkOrange, .333)));

    /* NEW CORAL PATHFINDS */
    driverController
        .rightBumper()
        .and(coralMode)
        .and(hasGamePiece)
        .and(driverPathOverride)
        .negate()
        .whileTrue(
            AutoAline.autoAlineToPath(this, stateController.getBranch())
                .andThen(lED.strobeCommand(Color.kDarkOrange, .333)));

    driverController
        .leftBumper()
        .and(coralMode)
        .and(hasGamePiece)
        .and(driverPathOverride)
        .negate()
        .whileTrue(
            AutoAline.autoAlineToPath(this, stateController.getBranch())
                .andThen(lED.strobeCommand(Color.kDarkOrange, .333)));

    /* INTAKE MODE */
    intakeMode.onTrue(superStructure.goToSource().alongWith(lED.strobeCommand(Color.kRed, 0.333)));

    intakeMode.onFalse(
        arm.home().alongWith(lED.solidCommand(Color.kGreen)).alongWith(manipulator.stopIntake()));

    hasGamePiece.onTrue(stateController.setNoIntakeMode());

    /* ALGAE PATHFINDS (left and right have no difference) */

    // Left bumper, algae mode, has piece -> processor
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

    // Left bumper, algae mode, no piece -> closest reef center face (same as right bumper)
    // driverController
    //     .leftBumper()
    //     .and(algaeMode)
    //     .and(hasNoGamePiece)
    //     .whileTrue(
    //         drive.defer(
    //             () ->
    //                 drive
    //                     .pathfindToFieldPose(AllianceFlipUtil.apply(drive.getClosestReefPanel()))
    //                     .andThen(lED.strobeCommand(Color.kDarkOrange, .333))));

    // Right bumper, algae mode, no piece -> closest reef center face (same as left bumper)
    // driverController
    //     .leftBumper()
    //     .and(algaeMode)
    //     .and(hasNoGamePiece)
    //     .whileTrue(
    //         drive.defer(
    //             () ->
    //                 drive
    //                     .pathfindToFieldPose(AllianceFlipUtil.apply(drive.getClosestReefPanel()))
    //                     .andThen(lED.strobeCommand(Color.kDarkOrange, .333))));

    /* CLIMBER PATHFIND */

    driverController
        .leftBumper()
        .or(driverController.rightBumper())
        .and(climbMode)
        .whileTrue(
            AutoAline.autoAlineToPath(this, stateController.getBranch())
                .andThen(lED.strobeCommand(Color.kDarkOrange, .333)));

    /* OPERATOR BUTTONS */

    // Set mode to coral
    operatorButtonBox.button(1).onTrue(stateController.setCoralMode(manipulator));

    // Set mode to algae
    operatorButtonBox.button(2).onTrue(stateController.setAlgaeMode(manipulator));

    // Set mode to climb when button coral and algae are pressed
    operatorButtonBox
        .button(1)
        .and(operatorButtonBox.button(2))
        .onTrue(stateController.setClimbMode(manipulator));

    // L4 Coral (queue)
    operatorButtonBox
        .button(3)
        .and(coralMode)
        .and(manualOverride)
        .negate()
        .onTrue(stateController.setL4());

    // L4 Coral (manual)
    operatorButtonBox
        .button(3)
        .and(coralMode)
        .and(manualOverride)
        .onTrue(
            arm.coralL4()
                .alongWith(
                    new ElevatorToSetpoint(elevator, ElevatorConstants.l4Pos)
                        .alongWith(stateController.setL4())));

    // L3 Coral (queue)
    operatorButtonBox
        .button(4)
        .and(coralMode)
        .and(manualOverride)
        .negate()
        .onTrue(stateController.setL3());

    // L3 Coral (manual)
    operatorButtonBox
        .button(4)
        .and(coralMode)
        .and(manualOverride)
        .onTrue(
            arm.coralL3()
                .alongWith(
                    new ElevatorToSetpoint(elevator, ElevatorConstants.l3Pos)
                        .alongWith(stateController.setL3())));

    // L2 Coral (queue)
    operatorButtonBox
        .button(5)
        .and(coralMode)
        .and(manualOverride)
        .negate()
        .onTrue(stateController.setL2());

    // L2 Coral (manual)
    operatorButtonBox
        .button(5)
        .and(coralMode)
        .and(manualOverride)
        .onTrue(
            arm.coralL2()
                .andThen(
                    new ElevatorToSetpoint(elevator, ElevatorConstants.l2Pos)
                        .alongWith(stateController.setL2())));

    // L1 Coral (queue)
    operatorButtonBox
        .button(6)
        .and(coralMode)
        .and(manualOverride)
        .negate()
        .onTrue(stateController.setL1());

    // L1 Coral (manual)
    operatorButtonBox
        .button(6)
        .and(coralMode)
        .and(manualOverride)
        .onTrue(
            arm.coralL1()
                .andThen(
                    new ElevatorToSetpoint(elevator, ElevatorConstants.l1Pos)
                        .alongWith(stateController.setL1())));

    // Algae barge
    // operatorButtonBox
    //     .button(3)
    //     .and(algaeMode)
    //     .onTrue(
    //         arm.barge()
    //             .andThen(new ElevatorToSetpoint(elevator, ElevatorConstants.l4Pos)
    //             .andThen(stateController.setL4())));

    // Removes algae l3
    operatorButtonBox
        .button(4)
        .and(algaeMode)
        .onTrue(
            new ElevatorToSetpoint(elevator, ElevatorConstants.l3PosAlgae)
                .alongWith(superStructure.removeL3Algae().alongWith(stateController.setL3())));

    // Removes algae l2
    operatorButtonBox
        .button(5)
        .and(algaeMode)
        .onTrue(
            superStructure
                .removeL2Algae()
                .alongWith(
                    new ElevatorToSetpoint(elevator, ElevatorConstants.l2PosAlgae)
                        .alongWith(stateController.setL2())));

    // Algae processor
    // operatorButtonBox
    //     .button(6)
    //     .and(algaeMode)
    //     .onTrue(
    //         arm.processor()
    //         .alongWith(new ElevatorToSetpoint(elevator, ElevatorConstants.l1Pos, true)
    //         .alongWith(stateController.setL1())));

    // Goes home (i love patrick mahomes)
    operatorButtonBox
        .button(7)
        .onTrue(
            new ElevatorToSetpoint(elevator, ElevatorConstants.homePos, true)
                .andThen(superStructure.goHome())
                .until(() -> !elevator.mahoming)
                .andThen(
                    elevator.runOnce(() -> elevator.stop()).andThen(stateController.setMahome())));

    // Deploy climber
    operatorButtonBox.button(8).onTrue(arm.climb());

    // Climb up
    // operatorButtonBox.button(9).and(climbMode).onTrue(climber.setClimberUp());

    // Intake
    operatorButtonBox
        .button(10)
        .whileTrue(
            superStructure
                .goToSource()
                .alongWith(
                    lED.strobeCommand(Color.kRed, 0.333)
                        .until(() -> manipulator.hasGamePiece())
                        .andThen(lED.solidCommand(Color.kGreen))));

    // Stop intake - begin manipulator power to hold coral
    operatorButtonBox
        .button(10)
        .onFalse(
            Commands.either(manipulator.keepCoralIn(), manipulator.stopIntake(), hasGamePiece)
                .andThen(arm.home())
                .alongWith(lED.defaultLeds(() -> stateController.getMode())));

    // Vomit
    operatorButtonBox
        .button(11)
        .whileTrue(
            manipulator
                .vomit()
                .alongWith(new ElevatorToSetpoint(elevator, ElevatorConstants.l2Pos)));

    operatorButtonBox
        .button(11)
        .onFalse(elevator.runOnce(() -> elevator.stop()).alongWith(manipulator.stopIntake()));

    // Fire
    operatorButtonBox.button(12).and(L1).onTrue(superStructure.fire());
    operatorButtonBox.button(12).and(L2).onTrue(superStructure.fire());
    operatorButtonBox.button(12).and(L3).onTrue(superStructure.fire());

    operatorButtonBox
        .button(12)
        .and(L4)
        .onTrue(
            superStructure.fire().until(() -> !manipulator.hasGamePiece()).andThen(arm.layup()));

    operatorButtonBox
        .button(12)
        .onFalse(
            superStructure
                .goHome()
                .andThen(
                    new ElevatorToSetpoint(elevator, ElevatorConstants.homePos, true)
                        .until(() -> !elevator.mahoming)
                        .andThen(elevator.runOnce(() -> elevator.stop()))
                        .alongWith(stateController.setMahome())));

    /* REEF BOX */
    operatorReefBox.button(7).onTrue(stateController.setBranch(Branch.BACK_LEFTBRANCH));
    operatorReefBox.button(8).onTrue(stateController.setBranch(Branch.BACK_RIGHTBRANCH));

    operatorReefBox.button(5).onTrue(stateController.setBranch(Branch.BACKLEFT_LEFTBRANCH));

    operatorReefBox.button(6).onTrue(stateController.setBranch(Branch.BACKLEFT_RIGHTBRANCH));
    operatorReefBox.button(9).onTrue(stateController.setBranch(Branch.BACKRIGHT_LEFTBRANCH));
    operatorReefBox.button(10).onTrue(stateController.setBranch(Branch.BACKRIGHT_RIGHTBRANCH));
    operatorReefBox.button(1).onTrue(stateController.setBranch(Branch.FRONT_LEFTBRANCH));
    operatorReefBox.button(2).onTrue(stateController.setBranch(Branch.FRONT_RIGHTBRANCH));
    operatorReefBox.button(3).onTrue(stateController.setBranch(Branch.FRONTLEFT_LEFTBRANCH));
    operatorReefBox.button(4).onTrue(stateController.setBranch(Branch.FRONTLEFT_RIGHTBRANCH));
    operatorReefBox.button(12).onTrue(stateController.setBranch(Branch.FRONTRIGHT_RIGHTBRANCH));
    operatorReefBox.button(11).onTrue(stateController.setBranch(Branch.FRONTRIGHT_LEFTBRANCH));

    // the reef is a lie

    /* TEST CONTROLLER */

    driverController.povUp().onTrue(climber.moveClimberUp());
    driverController.povUp().onFalse(climber.stop());

    driverController.povDown().onTrue(climber.moveClimberDown());
    driverController.povDown().onFalse(climber.stop());

    driverController.povRight().onTrue(climber.setPower(0.2));
    driverController.povRight().onFalse(climber.stop());

    // TODO: make lucas estel campau better
    // TODO: also make andrew john ferguson better
    // TODO: also also make gavin breezee better
    // TODO: also also also make mark zhang the seveteen thousandth better
    // TODO: also also also also also make richard ernest budop III better (jk)
    // TODO: also also also also also also make simon edward philips better

    testController.b().onTrue(arm.elbowUp());
    testController.b().onFalse(arm.stopElbow());

    testController.rightBumper().onTrue(arm.elbowDown());
    testController.rightBumper().onFalse(arm.stopElbow());

    // driverController.povDown().whileTrue(climber.setClimberHome());
    // driverController.povDown().onFalse(climber.stop());

    // testController.povLeft().onTrue(arm.wristDown());
    // testController.povLeft().onFalse(arm.stopWrist());

    // testController.y().onTrue(superStructure.moveElevatorUp());
    // testController.y().onFalse(superStructure.elevatorStop());

    // testController.leftBumper().onTrue(superStructure.moveElevatorDown());
    // testController.leftBumper().onFalse(superStructure.elevatorStop());

    // testController.leftTrigger().onTrue(superStructure.manipulatorOpen());
    // testController.leftTrigger().onFalse(superStructure.manipulatorStop());

    // testController.rightTrigger().onTrue(superStructure.manipulatorClose());
    // testController.rightTrigger().onFalse(superStructure.manipulatorStop());

    // testController.x().onTrue(superStructure.setWheelCurrent());
    // testController.x().onFalse(manipulator.stopIntake());

    // testController.leftBumper().onTrue(superStructure.runWheels());
    // testController.leftBumper().onFalse(manipulator.stopIntake());

    // testController.leftTrigger().onTrue(superStructure.moveElevatorUp());
    // testController.leftTrigger().onFalse(superStructure.elevatorStop());

    // testController.rightTrigger().onTrue(superStructure.moveElevatorDown());
    // testController.rightTrigger().onFalse(superStructure.elevatorStop());

    // testController.rightBumper().onTrue(superStructure.runWheelsBackwards());
    // testController.rightBumper().onFalse(manipulator.stopIntake());

    // testController.povRight().whileTrue(superStructure.moveWristUp());
    // testController.povRight().onFalse(superStructure.wristStop());

    // testController.a().whileTrue(superStructure.moveWristDown());
    // testController.a().onFalse(superStructure.wristStop());

    // testController.x().onTrue(climber.setClimberHome());
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
