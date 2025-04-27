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

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.FileVersionException;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.generated.TunerConstants;
import frc.robot.util.LocalADStarAK;
import java.io.IOException;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  // TunerConstants doesn't include these constants, so they are declared locally
  static final double ODOMETRY_FREQUENCY =
      new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  // PathPlanner config constants
  private static final double ROBOT_MASS_KG =
      61.235; // 74.088 old value, new from Scale is currently there
  private static final double ROBOT_MOI = 4.745; // 6.883 old value, new from CAD is currently there
  private static final double WHEEL_COF = 1.2;
  public boolean initializedGyro = false;

  private static final RobotConfig PP_CONFIG =
      new RobotConfig(
          ROBOT_MASS_KG,
          ROBOT_MOI,
          new ModuleConfig(
              TunerConstants.FrontLeft.WheelRadius,
              TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
              WHEEL_COF,
              DCMotor.getKrakenX60Foc(1)
                  .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
              60,
              1),
          getModuleTranslations());

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  private SwerveDrivePoseEstimator poseEstimatorNoVision =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    initializedGyro = false;
    modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
    modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
    modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController( // 2.5
            new PIDConstants(1.75, .0, 0.0, 0.0), new PIDConstants(7.0, 0.0, 0.0)),
        PP_CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    if (!initializedGyro && DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == Alliance.Blue) {
        poseEstimator.resetRotation(Rotation2d.fromDegrees(180));
        initializedGyro = true;
      } else {
        poseEstimator.resetRotation(Rotation2d.fromDegrees(0));
        initializedGyro = true;
      }
    }
    // Logger.recordOutput("initilized gyro", initializedGyro);

    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      // Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      // Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }
    // Logger.recordOutput("drive velo", modules[0].getVelocityRPS());

    // Logger.recordOutput("drive pos", modules[0].getPositionRotations());
    // Update gyro alert
    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

    // Log unoptimized setpoints and setpoint speeds
    // Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    // Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    // Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  public void runVeloOffset(ChassisSpeeds speeds, double radius) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates =
        kinematics.toSwerveModuleStates(discreteSpeeds, new Translation2d(radius, 0));
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

    // Log unoptimized setpoints and setpoint speeds
    // Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    // Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    // Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);

    // Logger.recordOutput("Modules 0 Velocity RPS", modules[0].getVelocityRPS());

    // Logger.recordOutput("Modules 0 Position Rotations", modules[0].getPositionRotations());
  }

  public Command drivethen(String pathName) {
    PathPlannerPath path;

    try {
      path = PathPlannerPath.fromPathFile(pathName);
      return AutoBuilder.pathfindThenFollowPath(
          PathPlannerPath.fromPathFile(pathName),
          new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720)));
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  public Command followPathCommand(String pathName, boolean firstPath)
      throws FileVersionException, IOException, org.json.simple.parser.ParseException {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    PathPlannerPath path2;
    boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    // Logger.recordOutput("Auto/isRed", isRed);

    if (isRed) path2 = path.flipPath();
    else path2 = path;

    // Logger.recordOutput("Auto/PathCurrentRun", path2.toString());
    if (firstPath) {
      // odometryLock.unlock();
      return Commands.runOnce(
              () ->
                  setPose(
                      new Pose2d(
                          path2.getPathPoses().get(0).getX(),
                          path2.getPathPoses().get(0).getY(),
                          Rotation2d.fromRadians(
                              path2
                                  .getPathPoses()
                                  .get(0)
                                  .getRotation()
                                  .getRadians()))), // + Math.PI used to be here, removed for
              // debugging purposes
              this)
          .andThen(AutoBuilder.followPath(path));
    } else return AutoBuilder.followPath(path);
  }

  public Command driveToPlayerStation() {
    boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    Pose2d playerStationPose =
        new Pose2d(1.846, 7.136, new Rotation2d(Units.degreesToRadians(128.)));
    if (isRed) {
      playerStationPose = new Pose2d(15.81, 0.782, new Rotation2d(Units.degreesToRadians(-48.)));
    }

    return AutoBuilder.pathfindToPose(
        playerStationPose,
        new PathConstraints(3, 4, Units.degreesToRadians(520), Units.degreesToRadians(720)));
  }

  public Command driveToPose(Supplier<Pose2d> pose) {
    return AutoBuilder.pathfindToPose(
        pose.get(),
        new PathConstraints(3, 4, Units.degreesToRadians(520), Units.degreesToRadians(720)));
  }

  public Command driveToNearestSide(Supplier<Pose2d> pose, BooleanSupplier isLeft) {
    // ensure NOTHING HAPPENS when pose null
    // if (Double.isNaN(pose.get().getX()) || Double.isNaN(pose.get().getX())) {
    // return Commands.run(
    // () -> {
    // int x = 5;
    // });
    // }
    Pose2d poseToReturn = pose.get();
    double leastMagnitude = 99;
    Pose2d currentPose = pose.get();
    AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    Pose3d[] reefTagPoses = new Pose3d[6];
    int offset = 6;
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Blue) {
      offset = 19;
    }
    for (int i = 0; i < 6; i++) {
      reefTagPoses[i] = layout.getTagPose(i + offset).get();
    }
    int minTag = 0;
    for (int i = 0; i < 6; i++) {
      Pose3d tagPose = reefTagPoses[i];

      double compMag =
          (currentPose
              .getTranslation()
              .getDistance(
                  new Pose2d(
                          tagPose.getX(),
                          tagPose.getY(),
                          Rotation2d.fromRadians(tagPose.getRotation().getAngle()))
                      .getTranslation()));

      if (compMag < leastMagnitude) {

        poseToReturn =
            new Pose2d(
                tagPose.getX(),
                tagPose.getY(),
                Rotation2d.fromRadians(tagPose.getRotation().getAngle() + Math.PI));
        leastMagnitude = compMag;
        minTag = i;
      }
    }
    // Logger.recordOutput("Vision/Nearest Siden PreOffset", poseToReturn);

    Pose2d tagOffset =
        new Pose2d(
            Units.inchesToMeters(40.), // GONNA MAKE THIS HUGE
            Units.inchesToMeters(isLeft.getAsBoolean() ? -6.5 : 6.5),
            new Rotation2d());
    tagOffset =
        tagOffset.rotateBy(Rotation2d.fromRadians(reefTagPoses[minTag].getRotation().getAngle()));

    poseToReturn =
        new Pose2d(
            poseToReturn.getX() + tagOffset.getX(),
            poseToReturn.getY() + tagOffset.getY(),
            poseToReturn.getRotation());

    // Logger.recordOutput("Vision/Nearest Side", poseToReturn);

    return AutoBuilder.pathfindToPose(
        poseToReturn,
        new PathConstraints(4.5, 14.5, Units.degreesToRadians(960), Units.degreesToRadians(960)));
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  // @AutoLogOutput(key = "Odometry/RobotKeinVision")
  // public Pose2d getPoseAuto() {
  // return poseEstimatorNoVision.getEstimatedPosition();
  // }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetTranslation(new Translation2d(pose.getX(), pose.getY()));
    // pose =
    // DriverStation.getAlliance().isPresent()
    // && DriverStation.getAlliance().get() == Alliance.Blue
    // ? pose
    // : new Pose2d(
    // pose.getX(),
    // pose.getY(),
    // Rotation2d.fromRadians(pose.getRotation().getRadians() + Math.PI));
    poseEstimator.resetRotation(pose.getRotation());
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);

    // poseEstimatorNoVision.resetTranslation(new Translation2d(pose.getX(),
    // pose.getY()));
    // pose =
    // DriverStation.getAlliance().isPresent()
    // && DriverStation.getAlliance().get() == Alliance.Blue
    // ? pose
    // : new Pose2d(
    // pose.getX(),
    // pose.getY(),
    // Rotation2d.fromRadians(pose.getRotation().getRadians() + Math.PI));
    // poseEstimatorNoVision.resetRotation(pose.getRotation());
    // poseEstimatorNoVision.resetPosition(rawGyroRotation, getModulePositions(),
    // pose);
  }

  // standardizing the teleop reset: climber is always front. Doing this for now
  // in case there's an
  // issue this causes via auto
  public void setPoseTeleop(Pose2d pose) {
    poseEstimator.resetTranslation(new Translation2d(pose.getX(), pose.getY()));
    pose =
        DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Blue
            ? pose
            : new Pose2d(
                pose.getX(),
                pose.getY(),
                Rotation2d.fromRadians(pose.getRotation().getRadians() + Math.PI));
    poseEstimator.resetRotation(pose.getRotation());
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);

    // poseEstimatorNoVision.resetTranslation(new Translation2d(pose.getX(),
    // pose.getY()));
    // pose =
    // DriverStation.getAlliance().isPresent()
    // && DriverStation.getAlliance().get() == Alliance.Blue
    // ? pose
    // : new Pose2d(
    // pose.getX(),
    // pose.getY(),
    // Rotation2d.fromRadians(pose.getRotation().getRadians() + Math.PI));
    // poseEstimatorNoVision.resetRotation(pose.getRotation());
    // poseEstimatorNoVision.resetPosition(rawGyroRotation, getModulePositions(),
    // pose);
  }

  public double getPitchAngularVelocity() {
    return gyroInputs.pitchVelocityRadPerSec;
  }
  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }
}
