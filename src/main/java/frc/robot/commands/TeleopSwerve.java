// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
// import frc.robot.subsystems.vision_ai.VisionAI;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead
https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TeleopSwerve extends Command {

  PIDConstants thetaConstants = new PIDConstants(10.0 / 5.0, 0.0, 0.0);
  PIDConstants xConstants = new PIDConstants(8.0, 0.0, 0.0);
  PIDConstants yConstants = new PIDConstants(8.0, 0.0, 0.0);
  TrapezoidProfile.Constraints thetaConstraints = new TrapezoidProfile.Constraints(5.0, 200.0);

  private final ProfiledPIDController angleController =
      new ProfiledPIDController(
          thetaConstants.kP, thetaConstants.kI, thetaConstants.kD, thetaConstraints);

  TrapezoidProfile.Constraints xConstraints = new TrapezoidProfile.Constraints(5.0, 200); // 200
  TrapezoidProfile.Constraints yConstraints = new TrapezoidProfile.Constraints(5.0, 200);
  private final ProfiledPIDController xController =
      new ProfiledPIDController(xConstants.kP, xConstants.kI, xConstants.kD, xConstraints);
  private final ProfiledPIDController yController =
      new ProfiledPIDController(yConstants.kP, yConstants.kI, yConstants.kD, yConstraints);

  private Drive drive;
  private DoubleSupplier xSupplier;
  private DoubleSupplier ySupplier;
  private DoubleSupplier omegaSupplier;
  // private VisionAI vision;

  private BooleanSupplier reefCentricDrive;

  private BooleanSupplier useSlowMode = () -> false;
  // Create PID controller
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 3.0;
  private static final double ANGLE_KD = 0.0; // .4
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 200.0;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  private double lastAngle = 0.0;
  private double lastTime = 999999.;

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband

    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude * linearMagnitude;
    Logger.recordOutput("Drive/Linear Magnitude", linearMagnitude);
    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  /** Creates a new TeleopSwerve. */
  public TeleopSwerve(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier useSlowMode,
      BooleanSupplier reefCentricDrive) {

    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.omegaSupplier = omegaSupplier;
    this.drive = drive;
    this.useSlowMode = useSlowMode;
    this.reefCentricDrive = reefCentricDrive;

    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastAngle = 0;
    lastTime = 999999.;
    angleController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double adjustmentFactor = 1.0;

    Translation2d linearVelocity =
        getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

    double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

    omega = 0.5 * Math.copySign(Math.pow(omega, 5), omega); // 0.5
    if (useSlowMode.getAsBoolean()) {
      if (Math.abs(linearVelocity.getX()) > 0.1 && Math.abs(linearVelocity.getY()) > 0.1) {
        linearVelocity =
            new Translation2d(linearVelocity.getX() * 0.1, linearVelocity.getY() * 0.1);
      } else {
        linearVelocity =
            new Translation2d(linearVelocity.getX() * 0.15, linearVelocity.getY() * 0.15);
      }

      omega = omega * 0.15;
    }

    // Convert to field relative speeds & send command
    if (!reefCentricDrive.getAsBoolean()) {
      ChassisSpeeds speeds =
          new ChassisSpeeds(
              linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
              linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
              omega * drive.getMaxAngularSpeedRadPerSec());
      boolean isFlipped =
          DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red;
      drive.runVelocity(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              speeds,
              isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
