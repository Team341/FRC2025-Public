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

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.motor_factories.elevator.Elevator;
import frc.robot.commands.ZeroCommands.ZeroCoral;
import frc.robot.commands.ZeroCommands.ZeroWrist;
import frc.robot.configs.MissDaisy;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.coral.Coral;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.subsystems.wrist.Wrist;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;
  private ZeroWrist wristZero;
  private ZeroCoral coralZero;

  public static boolean passivePulsing = false;
  public boolean throttled = true;

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
    LED.getInstance().clear();
    wristZero =
        new ZeroWrist(
            Wrist.getInstance(),
            Shoulder.getInstance(),
            Elevator.getInstance(new MissDaisy().getElevatorDescription()));
    coralZero = new ZeroCoral(Coral.getInstance());
    SmartDashboard.putNumber("Climber Setpoint", 79);

    NetworkTableInstance.getDefault()
        .getTable("limelight-climber")
        .getIntegerTopic("throttle_set")
        .publish()
        .set(0);
    NetworkTableInstance.getDefault()
        .getTable("limelight-right")
        .getIntegerTopic("throttle_set")
        .publish()
        .set(0);
    throttled = true;
    FollowPathCommand.warmupCommand().schedule();
    SignalLogger.enableAutoLogging(false);
    AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    field.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    RobotVisualizer.update();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    if (!LED.climbed) {
      passivePulsing = true;
    } else passivePulsing = false;
    LED.getInstance().clear();
    if (throttled) {
      if (DriverStation.isFMSAttached()) {
        NetworkTableInstance.getDefault()
            .getTable("limelight-climber")
            .getIntegerTopic("throttle_set")
            .publish()
            .set(0);
        NetworkTableInstance.getDefault()
            .getTable("limelight-right")
            .getIntegerTopic("throttle_set")
            .publish()
            .set(0);
        throttled = false;
      }
    } else {
      if (!DriverStation.isFMSAttached()) {
        NetworkTableInstance.getDefault()
            .getTable("limelight-climber")
            .getIntegerTopic("throttle_set")
            .publish()
            .set(0);
        NetworkTableInstance.getDefault()
            .getTable("limelight-right")
            .getIntegerTopic("throttle_set")
            .publish()
            .set(0);
        throttled = true;
      }
    }
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    if (throttled) {
      if (DriverStation.isFMSAttached()) {
        NetworkTableInstance.getDefault()
            .getTable("limelight-climber")
            .getIntegerTopic("throttle_set")
            .publish()
            .set(0);
        NetworkTableInstance.getDefault()
            .getTable("limelight-right")
            .getIntegerTopic("throttle_set")
            .publish()
            .set(0);
        throttled = false;
      }
    }
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    LED.aligning = false;
    LED.isAligned = false;
    if (throttled) {
      NetworkTableInstance.getDefault()
          .getTable("limelight-climber")
          .getIntegerTopic("throttle_set")
          .publish()
          .set(0);
      NetworkTableInstance.getDefault()
          .getTable("limelight-right")
          .getIntegerTopic("throttle_set")
          .publish()
          .set(0);
      throttled = false;
    }
    passivePulsing = false;
    LED.getInstance().clear();

    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }

    if (isFirst && Constants.currentMode != Constants.Mode.SIM) {
      coralZero.schedule();
      wristZero.schedule();
      isFirst = false;
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  boolean isFirst = true;

  @Override
  public void teleopInit() {
    LED.aligning = false;
    LED.isAligned = false;
    if (throttled) {
      NetworkTableInstance.getDefault()
          .getTable("limelight-climber")
          .getIntegerTopic("throttle_set")
          .publish()
          .set(0);
      NetworkTableInstance.getDefault()
          .getTable("limelight-right")
          .getIntegerTopic("throttle_set")
          .publish()
          .set(0);
      throttled = false;
    }

    passivePulsing = false;
    LED.getInstance().clear();

    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    if (isFirst && Constants.currentMode != Constants.Mode.SIM) {
      coralZero.schedule();
      wristZero.schedule();
      isFirst = false;
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    LED.getInstance().clear();

    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
