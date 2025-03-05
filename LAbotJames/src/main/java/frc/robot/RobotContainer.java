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
import frc.robot.subsystems.Manipulator.ManipulatorIOPhoenixRev;
import frc.robot.subsystems.StateController;
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
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
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
  private final CommandXboxController testController = new CommandXboxController(2);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Commands
  private Command goHome;

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
    stateController = new StateController();
    superStructure = new SuperStructure(manipulator, arm, elevator, stateController);

    lED.setDefaultCommand(lED.defaultLeds(() -> stateController.getMode()));

    // Set up auto routines

    // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

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
    NamedCommands.registerCommand("goToAlgae25", superStructure.goToL2Algae());
    NamedCommands.registerCommand("goToAlgae35", superStructure.goToL3Algae());
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
            .alongWith(superStructure.goToL3Algae().alongWith(stateController.setL3())));
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
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
    Trigger L1 = new Trigger(() -> stateController.isL1());
    Trigger L2 = new Trigger(() -> stateController.isL2());
    Trigger LMahome = new Trigger(() -> stateController.isMahome());
    Trigger L3 = new Trigger(() -> stateController.isL3());

    // rumble controler for 1s when endgame
    rumbleTime.onTrue(
        new InstantCommand(() -> driverController.setRumble(GenericHID.RumbleType.kRightRumble, .8))
            .andThen(new WaitCommand(1))
            .andThen(() -> driverController.setRumble(GenericHID.RumbleType.kRightRumble, 0)));

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    // angle towards reef panel if game piece
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

    // angle towards source if no game piece
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

    // angle towards processor if game piece and algae mode
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

    // angle towards reef if no game piece and algae mode
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
                () -> -driverController.getRightX()));

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

    // Slow Mode
    driverController
        .rightTrigger()
        .whileTrue(
            DriveCommands.joystickDrive(
                drive,
                () -> (-driverController.getLeftY() * .4),
                () -> (-driverController.getLeftX() * .4),
                () -> (-driverController.getRightX() * .4)));

    // Pathfinds based on State Controller

    // driverController
    //     .leftBumper()
    //     .and(coralMode)
    //     .and(hasGamePiece)
    //     .whileTrue(
    //         drive
    //             .defer(
    //                 () ->
    //                     drive.pathfindToFieldPose(
    //                         AllianceFlipUtil.apply(drive.getClosestReefBranch(true))))
    //             .andThen(lED.strobeCommand(Color.kDarkOrange, .333)));

    driverController
        .leftBumper()
        .and(coralMode)
        .and(hasGamePiece)
        .whileTrue(
            Commands.defer(
                    () ->
                        drive.pathfindToFieldPose(
                            () -> AllianceFlipUtil.apply(drive.getClosestReefBranch(true))),
                    Set.of(drive))
                .andThen(lED.strobeCommand(Color.kDarkOrange, .333)));

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
                .andThen(lED.strobeCommand(Color.kDarkOrange, .333)));

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

    driverController
        .rightBumper()
        .and(coralMode)
        .and(hasGamePiece)
        .whileTrue(
            Commands.defer(
                    () ->
                        drive.pathfindToFieldPose(
                            () -> AllianceFlipUtil.apply(drive.getClosestReefBranch(false))),
                    Set.of(drive))
                .andThen(lED.strobeCommand(Color.kDarkOrange, .333)));

    driverController
        .rightBumper()
        .and(coralMode)
        .and(hasNoGamePiece)
        .whileTrue(
            Commands.defer(
                () ->
                    drive
                        .pathfindToFieldPose(
                            () ->
                                AllianceFlipUtil.apply(
                                    FieldConstants.CoralStation.rightCenterIntakePos))
                        .andThen(lED.strobeCommand(Color.kDarkOrange, .333)),
                Set.of(drive)));

    // driverController
    //     .rightBumper()
    //     .and(algaeMode)
    //     .and(hasGamePiece)
    //     .whileTrue(
    //         drive.defer(
    //             () ->
    //                 drive
    //                     .pathfindToFieldPose(
    //                         AllianceFlipUtil.apply(FieldConstants.Processor.centerFace))
    //                     .andThen(lED.strobeCommand(Color.kDarkOrange, .333))));

    // driverController
    //     .rightBumper()
    //     .and(algaeMode)
    //     .and(hasNoGamePiece)
    //     .whileTrue(
    //         drive.defer(
    //             () ->
    //                 drive
    //                     .pathfindToFieldPose(AllianceFlipUtil.apply(drive.getClosestReefPanel()))
    //                     .andThen(lED.strobeCommand(Color.kDarkOrange, .333))));

    // Set mode to coral
    operatorButtonBox.button(1).onTrue(stateController.setCoralMode(manipulator));

    // Set mode to algae
    operatorButtonBox.button(2).onTrue(stateController.setAlgaeMode(manipulator));

    // Set mode to climb when button coral and algae are pressed
    operatorButtonBox
        .button(1)
        .and(operatorButtonBox.button(2))
        .onTrue(stateController.setClimbMode(manipulator));

    // goes to L4 positions
    operatorButtonBox
        .button(3)
        .and(coralMode)
        .onTrue(
            superStructure
                .goToL4Coral()
                .andThen(stateController.setL4())
                .andThen(new ElevatorToSetpoint(elevator, ElevatorConstants.l4Pos)));

    // goes to L3 positions
    operatorButtonBox
        .button(4)
        .and(coralMode)
        .onTrue(
            new ElevatorToSetpoint(elevator, ElevatorConstants.l3Pos)
                .alongWith(superStructure.goToL3Coral().alongWith(stateController.setL3())));

    // goes to L2 position
    operatorButtonBox
        .button(5)
        .and(coralMode)
        .onTrue(
            new ElevatorToSetpoint(elevator, ElevatorConstants.l2Pos)
                .alongWith(superStructure.goToL2Coral().alongWith(stateController.setL2())));

    // goes to L1 positions
    operatorButtonBox
        .button(6)
        .and(coralMode)
        .onTrue(
            new ElevatorToSetpoint(elevator, ElevatorConstants.l1Pos, true)
                .alongWith(superStructure.goToL1Coral())
                .alongWith(stateController.setL1()));

    // goes to L4 positions
    operatorButtonBox
        .button(3)
        .and(algaeMode)
        .onTrue(
            superStructure
                .goToL4Coral()
                .andThen(stateController.setL4())
                .andThen(new ElevatorToSetpoint(elevator, ElevatorConstants.l4Pos)));

    // goes to L3 positions
    operatorButtonBox
        .button(4)
        .and(algaeMode)
        .onTrue(
            new ElevatorToSetpoint(elevator, ElevatorConstants.l3PosAlgae)
                .alongWith(superStructure.goToL3Algae().alongWith(stateController.setL3())));

    // goes to L2 position
    operatorButtonBox
        .button(5)
        .and(algaeMode)
        .onTrue(
            new ElevatorToSetpoint(elevator, ElevatorConstants.l2PosAlgae)
                .alongWith(superStructure.goToL2Algae().alongWith(stateController.setL2())));

    // goes to L1 positions
    operatorButtonBox
        .button(6)
        .and(algaeMode)
        .onTrue(
            new ElevatorToSetpoint(elevator, ElevatorConstants.l1Pos, true)
                .alongWith(superStructure.goToL1Coral())
                .alongWith(stateController.setL1()));

    // goes home - i love patrick mahomes
    operatorButtonBox
        .button(7)
        .onTrue(
            new ElevatorToSetpoint(elevator, ElevatorConstants.homePos, true)
                .andThen(superStructure.intakeOff())
                .until(() -> !elevator.mahoming)
                .andThen(elevator.runOnce(() -> elevator.stop()))
                .andThen(superStructure.goHome().alongWith(stateController.setMahome())));

    // Deploy Climber
    operatorButtonBox
        .button(8)
        .and(climbMode)
        .onTrue(superStructure.armClimb().alongWith(climber.setClimberDown()));

    // Climbs
    operatorButtonBox.button(9).and(climbMode).onTrue(climber.setClimberUp());

    // Intake
    operatorButtonBox
        .button(10)
        .whileTrue(
            superStructure
                .goToSource()
                .alongWith(
                    lED.strobeCommand(Color.kRed, 0.32)
                        .until(() -> manipulator.hasGamePiece())
                        .andThen(lED.solidCommand(Color.kGreen))));

    operatorButtonBox
        .button(10)
        .onFalse(
            superStructure
                .intakeOff()
                .andThen(superStructure.goHome())
                .alongWith(lED.defaultLeds(() -> stateController.getMode())));

    // Vomit
    operatorButtonBox
        .button(11)
        .whileTrue(
            superStructure
                .vomit()
                .alongWith(new ElevatorToSetpoint(elevator, ElevatorConstants.l2Pos)));
    operatorButtonBox.button(11).onFalse(elevator.runOnce(() -> elevator.stop()));
    operatorButtonBox.button(11).onFalse(superStructure.intakeOff());

    // Fire
    operatorButtonBox
        .button(12)
        .and(L4)
        .onTrue(
            superStructure
                .fire()
                .until(() -> !manipulator.hasGamePiece())
                .andThen(superStructure.layup()));

    operatorButtonBox
        .button(12)
        .and(L4)
        .onFalse(
            superStructure
                .intakeOff()
                .andThen(
                    new ElevatorToSetpoint(elevator, ElevatorConstants.homePos, true)
                        .until(() -> !elevator.mahoming)
                        .andThen(elevator.runOnce(() -> elevator.stop()))
                        .alongWith(superStructure.goHome())));

    operatorButtonBox.button(12).and(L1).onTrue(superStructure.fire());

    operatorButtonBox
        .button(12)
        .and(L1)
        .onFalse(
            superStructure
                .intakeOff()
                .andThen(
                    new ElevatorToSetpoint(elevator, ElevatorConstants.homePos, true)
                        .until(() -> !elevator.mahoming)
                        .andThen(elevator.runOnce(() -> elevator.stop()))
                        .alongWith(superStructure.goHome())));

    operatorButtonBox.button(12).and(L2).onTrue(superStructure.fire());

    operatorButtonBox
        .button(12)
        .and(L2)
        .onFalse(
            superStructure
                .intakeOff()
                .andThen(
                    new ElevatorToSetpoint(elevator, ElevatorConstants.homePos, true)
                        .until(() -> !elevator.mahoming)
                        .andThen(elevator.runOnce(() -> elevator.stop()))
                        .alongWith(superStructure.goHome())));

    operatorButtonBox.button(12).and(L3).onTrue(superStructure.fire());

    operatorButtonBox
        .button(12)
        .and(L3)
        .onFalse(
            superStructure
                .intakeOff()
                .andThen(
                    new ElevatorToSetpoint(elevator, ElevatorConstants.homePos, true)
                        .until(() -> !elevator.mahoming)
                        .andThen(elevator.runOnce(() -> elevator.stop()))
                        .alongWith(superStructure.goHome())));

    // testController.a().onTrue(climber.moveClimberUp());
    // testController.a().onFalse(climber.stop());

    // testController.y().onTrue(climber.moveClimberDown());
    // testController.y().onFalse(climber.stop());

    // testController.b().onTrue(arm.elbowUp());
    // testController.b().onFalse(arm.stopElbow());

    // testController.rightBumper().onTrue(arm.elbowDown());
    // testController.rightBumper().onFalse(arm.stopElbow());

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
    // testController.x().onFalse(superStructure.intakeOff());

    // testController.leftBumper().onTrue(superStructure.runWheels());
    // testController.leftBumper().onFalse(superStructure.intakeOff());

    // testController.leftTrigger().onTrue(superStructure.moveElevatorUp());
    // testController.leftTrigger().onFalse(superStructure.elevatorStop());

    // testController.rightTrigger().onTrue(superStructure.moveElevatorDown());
    // testController.rightTrigger().onFalse(superStructure.elevatorStop());

    // testController.b().onTrue(elevator.elevatorQuasiUp());
    // testController.b().onFalse(superStructure.elevatorStop());

    // testController.a().onTrue(elevator.elevatorQuasiDown());
    // testController.a().onFalse(superStructure.elevatorStop());

    // testController.y().onTrue(elevator.elevatorDynaUp());
    // testController.y().onFalse(superStructure.elevatorStop());

    // testController.x().onTrue(elevator.elevatorDynaDown());
    // testController.x().onFalse(superStructure.elevatorStop());

    // testController.rightBumper().onTrue(superStructure.runWheelsBackwards());
    // testController.rightBumper().onFalse(superStructure.intakeOff());

    // testController.a().onTrue(new ElevatorToSetpoint(elevator, ElevatorConstants.l2Pos));

    // testController.start().whileTrue(arm.elbowDynaForward());

    // testController.back().whileTrue(arm.elbowDynaBackward());

    // testController.leftStick().whileTrue(arm.elbowQuasiBackward());

    // testController.rightStick().whileTrue(arm.elbowQuasiForward());

    // testController.povDown().whileTrue(superStructure.goToL1Coral());

    // testController.rightStick().whileTrue(superStructure.goToL2Algae());

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

// andrew is super awesome and cool
