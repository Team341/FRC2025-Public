// Copyright 2021-2024 FRC 6328
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
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.motor_factories.elevator.Elevator;
import frc.robot.commands.AlignToNearestPoseCommand;
import frc.robot.commands.AlignToNearestPoseCommand.AlignmentDirection;
import frc.robot.commands.Autonomous.Fling;
// import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FindReefOffsets;
import frc.robot.commands.FixStuck;
// import frc.robot.commands.TeleopCommands.DriveToPieceAI;
import frc.robot.commands.TeleopCommands.GoToPositionCommand;
import frc.robot.commands.TeleopCommands.ScoreBack;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.ZeroCommands.ZeroClimber;
import frc.robot.commands.ZeroCommands.ZeroWrist;
import frc.robot.configs.RobotConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.SuperStructure.RobotPose;
import frc.robot.subsystems.SuperStructure.RobotPoseDefinitions;
import frc.robot.subsystems.SuperStructure.Superstructure;
import frc.robot.subsystems.SuperStructure.Superstructure.robotState;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.coral.Coral;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
// import frc.robot.subsystems.vision_ai.VisionAI;
import frc.robot.subsystems.wrist.Wrist;
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
  private final Vision vision;
  private final Drive drive;
  private final Gripper gripper;
  private final Coral coral;
  private final Elevator elevator;
  private final Wrist wrist;
  private final Shoulder shoulder;
  // private final Algae algae;
  private final Superstructure struct;
  private final Climber climber;
  // private final VisionAI visionAI;
  public static boolean coralNotZeroed = false;
  // Controller

  private final AprilTagFieldLayout layout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);
  private final CommandXboxController programming = new CommandXboxController(2);
  private Command verticalIntakeCommand;
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final RobotConstants robotConfig;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    coral = Coral.getInstance();
    robotConfig = RobotConstants.getRobotConstants(Constants.identity);
    elevator = Elevator.getInstance(robotConfig.getElevatorDescription());
    shoulder = Shoulder.getInstance();
    gripper = Gripper.getInstance(shoulder.getAngleSupplier());
    wrist = Wrist.getInstance();
    struct = Superstructure.getInstance();
    climber = Climber.getInstance();
    wrist.resetAngle(Units.degreesToRadians(-102.));
    coral.resetPosition(Math.PI / 2);
    struct.hasPiece = gripper.hasPiece();
    elevator.resetPosition(0);
    RobotVisualizer.addCoralAngle(() -> coral.getPivotAngle());
    RobotVisualizer.addElevatorHeight(() -> elevator.getPosition());
    RobotVisualizer.addWristAngle(() -> wrist.getAngle());
    RobotVisualizer.addShoulderAngle(() -> shoulder.getAngle());

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
                () -> LED.aligning,
                drive::addVisionMeasurement,
                new VisionIOLimelight(camera0Name, drive::getRotation),
                new VisionIOLimelight(camera1Name, drive::getRotation),
                new VisionIOLimelight(camera2Name, drive::getRotation));

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
                () -> LED.aligning,
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));

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

        // (Use same number of dummy implementations as the real robot)
        vision =
            new Vision(
                () -> LED.aligning,
                drive::addVisionMeasurement,
                new VisionIO() {},
                new VisionIO() {});

        break;
    }
    verticalIntakeCommand =
        struct
            .createStateCommand(robotState.VERTICAL_INTAKE)
            .alongWith(
                Commands.run(
                    () -> {
                      gripper.runVolts(() -> -12);
                      struct.isFlipped = false;
                    },
                    gripper))
            .until(gripper.hasPiece().debounce(.5))
            .finallyDo(
                () -> {
                  gripper.runVolts(() -> 0);
                  struct.isFlipped = true;
                  struct.setDesiredState(robotState.STOW);
                })
            .withTimeout(7.5);

    NamedCommands.registerCommand(
        "algaeSwap",
        new ParallelCommandGroup(
            new InstantCommand(
                () -> {
                  struct.isAlgae = true;
                }),
            new InstantCommand(() -> drive.runVelocity(new ChassisSpeeds()))));
    NamedCommands.registerCommand(
        "Align Left",
        new AlignToNearestPoseCommand(drive, AlignmentDirection.LEFT).withTimeout(3.5));

    NamedCommands.registerCommand(
        "Align Coral", new AlignToNearestPoseCommand(drive, AlignmentDirection.CORAL));
    NamedCommands.registerCommand("Vertical Intake", verticalIntakeCommand);

    NamedCommands.registerCommand(
        "Flip",
        Commands.runOnce(
            () -> {
              Superstructure.isFlipped = !Superstructure.isFlipped;
            }));

    // CHANGE TO WORK FOR RED?
    NamedCommands.registerCommand(
        "Auto Start Align Left",
        new AlignToNearestPoseCommand(drive, AlignmentDirection.LEFT, true, true).withTimeout(3.));
    NamedCommands.registerCommand(
        "Auto Start Align Right",
        new AlignToNearestPoseCommand(drive, AlignmentDirection.LEFT, true, false).withTimeout(3.));

    NamedCommands.registerCommand(
        "Align Right",
        new AlignToNearestPoseCommand(drive, AlignmentDirection.RIGHT).withTimeout(3.5));

    NamedCommands.registerCommand(
        "Align Processor",
        new AlignToNearestPoseCommand(drive, AlignmentDirection.PROCESSOR).withTimeout(3.));

    NamedCommands.registerCommand(
        "Align Center",
        new AlignToNearestPoseCommand(drive, AlignmentDirection.CENTER).withTimeout(7.));

    NamedCommands.registerCommand(
        "Align to HP", new AlignToNearestPoseCommand(drive, AlignmentDirection.HP).withTimeout(3.));

    NamedCommands.registerCommand(
        "DriveIntoWall",
        new InstantCommand(
            () ->
                drive.runVelocity(
                    ChassisSpeeds.fromRobotRelativeSpeeds(
                        new ChassisSpeeds(0, -1, 0), drive.getRotation())),
            drive));

    NamedCommands.registerCommand(
        "Intake Ground",
        Commands.runEnd(
                () -> {
                  struct.setDesiredState(robotState.GROUND_INTAKE);
                  if (MathUtil.isNear(
                      RobotPoseDefinitions.GROUND_INTAKE.getShoulderAngle(),
                      shoulder.getAngle(),
                      Units.degreesToRadians(4))) gripper.runVolts(() -> -12);
                },
                () -> {
                  //   struct.setDesiredState(robotState.STOW);

                  gripper.runVolts(() -> 0); // Stop gripper
                })
            .until(gripper.hasPiece()));
    NamedCommands.registerCommand("Auto Stow", struct.createStateCommand(robotState.INITIAL_STOW));
    NamedCommands.registerCommand("Algae Fling", struct.createStateCommand(robotState.ALGAE_FLING));

    NamedCommands.registerCommand(
        "Intake HP",
        Commands.runEnd(
                () -> {
                  struct.isFront = false;
                  struct.setDesiredState(robotState.PLAYER_INTAKE);
                  if (MathUtil.isNear(
                      RobotPoseDefinitions.PLAYER_INTAKE.getShoulderAngle(),
                      shoulder.getAngle(),
                      Units.degreesToRadians(5))) gripper.runVolts(() -> -12);
                },
                () -> {
                  struct.setDesiredState(robotState.INITIAL_STOW);
                  gripper.runVolts(() -> 0); // Stop gripper
                })
            .until(gripper.hasPiece().debounce(.2))
            .withTimeout(3.5));

    NamedCommands.registerCommand("Fling", new Fling(gripper, struct));

    NamedCommands.registerCommand("Stow", struct.createStateCommand(robotState.INITIAL_STOW));
    NamedCommands.registerCommand(
        "Stow for Algae", struct.createStateCommand(robotState.STOW_FOR_ALGAE_AUTO));

    NamedCommands.registerCommand(
        "Algae Stow", struct.createStateCommand(robotState.ALGAE_HOLDING));

    NamedCommands.registerCommand(
        "L1", struct.createStateCommand(robotState.L1_PRESCORE).withTimeout(1.5));

    NamedCommands.registerCommand(
        "Stop", new InstantCommand(() -> drive.runVelocity(new ChassisSpeeds()), drive));

    NamedCommands.registerCommand(
        "L2", struct.createStateCommand(robotState.L2_PRESCORE).withTimeout(1.5));
    NamedCommands.registerCommand(
        "L3", struct.createStateCommand(robotState.L3_PRESCORE).withTimeout(1.5));
    NamedCommands.registerCommand(
        "L4", struct.createStateCommand(robotState.L4_PRESCORE).withTimeout(1.5));
    NamedCommands.registerCommand(
        "Algae Go To Score", struct.createStateCommand(robotState.ALGAE_GROUND_INTAKE));

    NamedCommands.registerCommand(
        "Algae High", struct.createStateCommand(robotState.L3_ALGAE_GRAB));
    NamedCommands.registerCommand("Algae Low", struct.createStateCommand(robotState.L2_ALGAE_GRAB));

    NamedCommands.registerCommand(
        "Intake",
        Commands.runEnd(
                () -> {
                  gripper.runVolts(() -> -12);
                },
                () -> {
                  struct.setDesiredState(robotState.INITIAL_STOW);
                  gripper.runVolts(() -> 0); // Stop grippe
                })
            .until(gripper.hasPiece()));

    NamedCommands.registerCommand(
        "Intake Algae",
        Commands.runEnd(
                () -> {
                  gripper.runVolts(() -> -12);
                },
                () -> {
                  //   struct.setDesiredState(robotState.L2_ALGAE_GRAB);
                  gripper.runVolts(() -> -4); // Stop gripper
                })
            .until(gripper.hasPiece()));

    NamedCommands.registerCommand(
        "Score Algae",
        Commands.defer(
                () -> {
                  robotState targetState =
                      switch (struct.desiredState) {
                        case L1_PRESCORE -> robotState.L1_SCORE;
                        case L2_PRESCORE -> robotState.L2_SCORE;
                        case L3_PRESCORE -> robotState.L3_SCORE;
                        case L4_PRESCORE -> robotState.L4_SCORE;
                        case L2_ALGAE_GRAB -> robotState.ALGAE_GROUND_INTAKE;
                        default -> struct.desiredState;
                      };
                  return struct
                      .createStateCommand(targetState)
                      .alongWith(Commands.run(() -> gripper.runVolts(() -> -1)))
                      .finallyDo(() -> gripper.runVolts(() -> 4))
                      .andThen(Commands.run(() -> {})); // Keep alive
                },
                Set.of(gripper))
            .finallyDo(
                () -> {
                  struct.setDesiredState(robotState.STOW);
                  gripper.runVolts(() -> 0);
                })
            .withTimeout(1.));

    NamedCommands.registerCommand(
        "Score Deploy",
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                new ScoreBack(drive, struct, gripper, false).withTimeout(0.75),
                new ScoreBack(drive, struct, gripper, true)),
            coral.setPivotAngle(() -> 0)));

    NamedCommands.registerCommand(
        "Score",
        new SequentialCommandGroup(
            new WaitCommand(0.4),
            new ScoreBack(drive, struct, gripper, false).withTimeout(0.75),
            new ScoreBack(drive, struct, gripper, true)));

    NamedCommands.registerCommand(
        "Score No Delay",
        new SequentialCommandGroup(
            new ScoreBack(drive, struct, gripper, false).withTimeout(0.75),
            new ScoreBack(drive, struct, gripper, true)));

    NamedCommands.registerCommand(
        "Score Final",
        new SequentialCommandGroup(
            new WaitCommand(0.25),
            new ScoreBack(drive, struct, gripper, false, true).withTimeout(0.75),
            new ScoreBack(drive, struct, gripper, true, true)));
    NamedCommands.registerCommand("Run Gripper", Commands.run(() -> gripper.runVolts(() -> 12)));

    NamedCommands.registerCommand("Coral Deploy", coral.setPivotAngle(() -> 0).withTimeout(0.35));
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    // autoChooser.addOption("2,4", new TwoPointFiveTop(drive, struct));
    // autoChooser.addOption("FourL4", new PathPlannerAuto("FourL4"));
    // autoChooser.addOption("FourL4Ground", new
    // PathPlannerAuto("FourL4GroundRight"));
    // autoChooser.addOption("FourL4Left", new PathPlannerAuto("FourL4Left"));
    autoChooser.addOption("FourL4LeftAlt", new PathPlannerAuto("FourL4LeftAlt"));

    // autoChooser.addOption("Four Meters Forward", new PathPlannerAuto("3 Meters
    // Forward"));
    autoChooser.addOption("Center Path", new PathPlannerAuto("CenterAuto"));
    // autoChooser.addOption("Drive By", new PathPlannerAuto("driveby"));
    autoChooser.addOption("FourL4RightAlt", new PathPlannerAuto("FourL4RightAlt"));
    autoChooser.addOption("3CRight", new PathPlannerAuto("3CRight"));
    autoChooser.addOption("CCLG RL", new PathPlannerAuto("CCLG right-left"));
    autoChooser.addOption("CCLG LL", new PathPlannerAuto("CCLG left-left"));

    autoChooser.addOption("CCLG LR", new PathPlannerAuto("CCLG left-right"));

    autoChooser.addOption("CCLG RR", new PathPlannerAuto("CCLG right-right"));

    // autoChooser.addOption("FourL4RightDriveIn", new
    // PathPlannerAuto("FourL4WallDrive"));

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));

    // autoChooser.addOption(
    // "Drive Simple FF Characterization",
    // DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    // "Drive SysId (Quasistatic Forward)",
    // drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    // "Drive SysId (Quasistatic Reverse)",
    // drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    // "Drive SysId (Dynamic Forward)",
    // drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    // "Drive SysId (Dynamic Reverse)",
    // drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption("Test", new PathPlannerAuto("Test"));
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    // -------------- DEFAULT COMMANDS ----------------

    shoulder.setDefaultCommand(
        Commands.run(
            () ->
                shoulder.runAngleDynamic(
                    struct
                        .getDesiredPose(
                            new RobotPose(
                                elevator.getPosition(),
                                shoulder.getAngle(),
                                wrist.getAngle(),
                                coral.getPivotAngle()))
                        .getShoulderAngle(),
                    () ->
                        struct
                            .getDesiredPose(
                                new RobotPose(
                                    elevator.getPosition(),
                                    shoulder.getAngle(),
                                    wrist.getAngle(),
                                    coral.getPivotAngle()))
                            .getShoulderConfigs()),
            shoulder));

    elevator.setDefaultCommand(
        Commands.run(
            () ->
                elevator.setHeightDynamic(
                    () ->
                        struct
                            .getDesiredPose(
                                new RobotPose(
                                    elevator.getPosition(),
                                    shoulder.getAngle(),
                                    wrist.getAngle(),
                                    coral.getPivotAngle()))
                            .getElevatorHeight(),
                    () ->
                        struct
                            .getDesiredPose(
                                new RobotPose(
                                    elevator.getPosition(),
                                    shoulder.getAngle(),
                                    wrist.getAngle(),
                                    coral.getPivotAngle()))
                            .getElevatorConfigs()),
            elevator));

    wrist.setDefaultCommand(
        wrist.setPosition(
            () ->
                struct
                    .getDesiredPose(
                        new RobotPose(
                            elevator.getPosition(),
                            shoulder.getAngle(),
                            wrist.getAngle(),
                            coral.getPivotAngle()))
                    .getWristAngle()));

    coral.setDefaultCommand(
        new RunCommand(
            () ->
                coral.setIntakeVoltageandSetPivotAngle(
                    () ->
                        struct
                            .getDesiredPose(
                                new RobotPose(
                                    elevator.getPosition(),
                                    shoulder.getAngle(),
                                    wrist.getAngle(),
                                    coral.getPivotAngle()))
                            .getCoralIntakeAngle(),
                    () ->
                        struct
                            .getDesiredPose(
                                new RobotPose(
                                    elevator.getPosition(),
                                    shoulder.getAngle(),
                                    wrist.getAngle(),
                                    coral.getPivotAngle()))
                            .getCoralIntakeSpeed()),
            coral));
    drive.setDefaultCommand(
        new TeleopSwerve(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX(),
            () -> controller.leftBumper().getAsBoolean(),
            () -> controller.b().getAsBoolean()));

    // ----------------------DRIVER CONTROLS--------------------------
    // DO NOT TOUCH OR COMMENT OUT, I'M LOOKING AT YOU GAO
    gripper
        .hasPiece()
        .onChange(
            Commands.runEnd(
                    () -> controller.setRumble(RumbleType.kBothRumble, 1.0),
                    () -> controller.setRumble(RumbleType.kBothRumble, .0))
                .withTimeout(.5));
    controller
        .y()
        .whileTrue(new AlignToNearestPoseCommand(drive, AlignmentDirection.HP))
        .onFalse(
            new InstantCommand(
                () -> {
                  LED.aligning = false;
                }));
    controller
        .leftTrigger(.1)
        .whileTrue(
            new AlignToNearestPoseCommand(
                drive, struct.isAlgae ? AlignmentDirection.CENTER : AlignmentDirection.LEFT))
        .onFalse(
            new InstantCommand(
                () -> {
                  LED.aligning = false;
                  LED.isAligned = false;
                }));

    controller
        .rightTrigger(.1)
        .whileTrue(
            new AlignToNearestPoseCommand(
                drive, struct.isAlgae ? AlignmentDirection.CENTER : AlignmentDirection.RIGHT))
        .onFalse(
            new InstantCommand(
                () -> {
                  LED.aligning = false;
                  LED.isAligned = false;
                }));

    controller.back().whileTrue(new FixStuck(elevator, coral, shoulder, wrist, gripper));

    controller
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPoseTeleop(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    programming.rightBumper().whileTrue(new ScoreBack(drive, struct, gripper, false, true));

    programming.leftStick().whileTrue(new Fling(gripper, struct));

    programming.a().whileTrue(new FindReefOffsets(drive));

    programming
        .rightStick()
        .whileTrue(new RunCommand(() -> gripper.runVolts(() -> 12), gripper))
        .onFalse(new RunCommand(() -> gripper.runVolts(() -> 0), gripper));

    programming.b().whileTrue(new ZeroClimber(climber, elevator));

    programming
        .rightTrigger(0.1)
        .whileTrue(
            Commands.run(
                () -> {
                  struct.setDesiredState(robotState.PROCESSOR);
                  if (MathUtil.isNear(
                      struct
                          .getDesiredPose(
                              new RobotPose(
                                  elevator.getPosition(),
                                  shoulder.getAngle(),
                                  wrist.getAngle(),
                                  coral.getPivotAngle()))
                          .getShoulderAngle(),
                      shoulder.getAngle(),
                      Units.degreesToRadians(4))) gripper.runVolts(() -> 4);
                }));

    programming.y().onTrue(Commands.runOnce(() -> struct.setAlgae(!struct.isAlgae())));

    // ---------------------- OPERATOR BINDINGS ------------------
    operatorController
        .leftTrigger(0.2)
        .whileTrue(
            Commands.runEnd(
                    () -> {
                      struct.setDesiredState(robotState.GROUND_INTAKE);
                      if (MathUtil.isNear(
                          RobotPoseDefinitions.GROUND_INTAKE.getShoulderAngle(),
                          shoulder.getAngle(),
                          Units.degreesToRadians(4))) gripper.runVolts(() -> -12);

                      if (struct.isAlgae
                          && MathUtil.isNear(
                              RobotPoseDefinitions.ALGAE_GROUND_INTAKE.getShoulderAngle(),
                              shoulder.getAngle(),
                              Units.degreesToRadians(4))) {
                        gripper.runVolts(() -> -12);
                      }
                    },
                    () -> {
                      struct.setDesiredState(robotState.STOW);

                      gripper.runVolts(() -> struct.isAlgae ? -8 : 0); // Stop gripper
                    })
                .until(
                    () -> {
                      if (!struct.isAlgae) {
                        return gripper.hasPiece().getAsBoolean();
                      } else {
                        return false;
                      }
                    }));

    operatorController
        .rightTrigger(0.2)
        .whileTrue(
            new ParallelCommandGroup(
                Commands.runEnd(
                    () -> {
                      struct.setDesiredState(robotState.GROUND_INTAKE);
                      if (MathUtil.isNear(
                          RobotPoseDefinitions.GROUND_INTAKE.getShoulderAngle(),
                          shoulder.getAngle(),
                          Units.degreesToRadians(4))) gripper.runVolts(() -> -12);
                    },
                    () -> {
                      gripper.runVolts(() -> 0); // Stop gripper
                    }),
                Commands.run(
                    () -> coral.setIntakeVoltageandSetPivotAngle(() -> 0, () -> -0.25), coral)));

    operatorController
        .a()
        .whileTrue(new GoToPositionCommand(struct, gripper, robotState.STOW, false));
    operatorController
        .b()
        .whileTrue(
            new GoToPositionCommand(struct, gripper, robotState.L1_PRESCORE, true)
                .andThen(new GoToPositionCommand(struct, gripper, robotState.L1_PRESCORE, false)))
        .onFalse(new InstantCommand(() -> struct.setDesiredState(robotState.L1_PRESCORE)));
    operatorController
        .x()
        .whileTrue(
            new SequentialCommandGroup(
                new GoToPositionCommand(struct, gripper, robotState.L2_PRESCORE, true),
                new GoToPositionCommand(struct, gripper, robotState.L2_PRESCORE, false)))
        .onFalse(
            new InstantCommand(
                () ->
                    struct.setDesiredState(
                        struct.isAlgae ? robotState.L2_ALGAE_GRAB : robotState.L2_PRESCORE)));
    ;

    operatorController
        .y()
        .whileTrue(
            new SequentialCommandGroup(
                new GoToPositionCommand(struct, gripper, robotState.L3_PRESCORE, true),
                new GoToPositionCommand(struct, gripper, robotState.L3_PRESCORE, false)))
        .onFalse(
            new InstantCommand(
                () ->
                    struct.setDesiredState(
                        struct.isAlgae ? robotState.L3_ALGAE_GRAB : robotState.L3_PRESCORE)));
    ;

    operatorController
        .rightBumper()
        .and(() -> !struct.isAlgae)
        .whileTrue(
            new SequentialCommandGroup(
                new GoToPositionCommand(struct, gripper, robotState.L4_PRESCORE, true),
                new GoToPositionCommand(struct, gripper, robotState.L4_PRESCORE, false)))
        .onFalse(new InstantCommand(() -> struct.setDesiredState(robotState.L4_PRESCORE)));
    ;

    operatorController
        .leftBumper()
        .and(() -> struct.isAlgae)
        .whileTrue(struct.createStateCommand(robotState.ALGAE_FLING));

    controller
        .rightBumper()
        .whileTrue(
            new SequentialCommandGroup(
                new ScoreBack(drive, struct, gripper, false).withTimeout(0.75),
                new ScoreBack(drive, struct, gripper, true)));
    operatorController
        .start()
        .onTrue(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                        Commands.run(() -> struct.setDesiredState(robotState.CLIMB_POST))
                            .withTimeout(0.4))
                    .withTimeout(1.),
                climber.setAngle(
                    () ->
                        climber.getLimit()
                            ? Units.radiansToDegrees(climber.getAngle())
                            : ClimberConstants.CLIMBER_POST_ANGLE)))
        .onFalse(
            new InstantCommand(
                () -> {
                  LED.climbed = true;
                }));

    operatorController
        .back()
        .onTrue(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                        Commands.runOnce(
                                () -> {
                                  struct.climbing = true;
                                })
                            .withTimeout(0.4))
                    .withTimeout(1.),
                climber.setAngle(() -> 165.)))
        .onFalse(
            new InstantCommand(
                () -> {
                  struct.climbing = false;
                }));

    operatorController
        .leftBumper()
        .and(() -> !struct.isAlgae)
        .whileTrue(
            Commands.runEnd(
                    () -> {
                      struct.setDesiredState(robotState.PLAYER_INTAKE);
                      if (MathUtil.isNear(
                          RobotPoseDefinitions.PLAYER_INTAKE.getShoulderAngle(),
                          shoulder.getAngle(),
                          Units.degreesToRadians(5))) gripper.runVolts(() -> -12);
                      if (!struct.isFront
                          && MathUtil.isNear(
                              RobotPoseDefinitions.PLAYER_INTAKE.getShoulderAngle(),
                              -shoulder.getAngle(),
                              Units.degreesToRadians(5))) gripper.runVolts(() -> -12);
                    },
                    () -> {
                      struct.setDesiredState(robotState.STOW);
                      gripper.runVolts(() -> 0); // Stop gripper
                    })
                .until(gripper.hasPiece()))
        .onTrue(
            new InstantCommand(
                () -> {
                  LED.blinking = true;
                }))
        .onFalse(
            new InstantCommand(
                () -> {
                  LED.blinking = false;
                  LED.getInstance().clearAnimation();
                }));
    operatorController
        .povCenter()
        .onTrue(Commands.runOnce(() -> struct.setAlgae(!struct.isAlgae())));

    operatorController.leftStick().onTrue(new ZeroWrist(wrist, shoulder, elevator));

    operatorController
        .rightStick()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      Superstructure.firstFlippedPose = true;
                    })
                .andThen(struct.createStateCommand2(() -> struct.getCurrentState()))
                .andThen(
                    Commands.runOnce(
                            () -> {
                              Superstructure.isFlipped = !Superstructure.isFlipped;
                            })
                        .andThen(new WaitCommand(0.5))
                        .andThen(
                            Commands.runOnce(
                                () -> {
                                  Superstructure.firstFlippedPose = false;
                                }))));
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
