// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SuperStructure.RobotPoseDefinitions;
import frc.robot.subsystems.SuperStructure.Superstructure;
import frc.robot.subsystems.SuperStructure.Superstructure.robotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.gripper.Gripper;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ScoreBack extends Command {
  private Drive drive;
  private Superstructure struct;
  private Gripper gripper;
  private robotState targetState;
  private Pose2d startingPose;
  private double tol = 7.;
  private boolean score;
  private boolean autoFinale = false;

  /** Creates a new ScoreBack. */
  public ScoreBack(Drive drive, Superstructure struct, Gripper gripper, boolean score) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.score = score;
    this.gripper = gripper;
    this.struct = struct;
    addRequirements(gripper, drive);
  }

  public ScoreBack(
      Drive drive, Superstructure struct, Gripper gripper, boolean score, boolean autoFinale) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.score = score;
    this.gripper = gripper;
    this.struct = struct;
    this.autoFinale = autoFinale;
    addRequirements(gripper, drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetState =
        switch (struct.desiredState) {
          case L1_PRESCORE -> robotState.L1_PRESCORE;
          case L2_PRESCORE -> robotState.L2_SCORE;
          case L3_PRESCORE -> robotState.L3_SCORE;
          case L4_PRESCORE -> robotState.L4_SCORE;
          case ALGAE_FLING -> robotState.ALGAE_FLINGUP;
          default -> {
            yield struct.desiredState;
          }
        };
    if (autoFinale) {
      targetState = robotState.L4_FINALE;
    }
    if (targetState == robotState.ALGAE_FLINGUP && gripper.notifierCalled()) {
      struct.setDesiredState(targetState);
    }

    if (targetState == robotState.ALGAE_FLINGUP) {
      gripper.setFling(true);
      drive.stopWithX();
    }
    ;
    startingPose = drive.getPose();

    if (targetState == robotState.L4_SCORE) {
      tol = 10;
    } else if (targetState == robotState.L3_SCORE) {
      tol = 7;
    } else if (targetState == robotState.L2_SCORE) {
      tol = 7;
    }
    if (targetState == robotState.L4_FINALE) {
      tol = 10;
    }
    if (targetState != robotState.ALGAE_FLINGUP) {
      struct.setDesiredState(targetState);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (targetState == robotState.ALGAE_FLINGUP && gripper.notifierCalled()) {
      struct.setDesiredState(targetState);
    }
    if (((MathUtil.isNear(
                Units.radiansToDegrees(
                    struct.getOriginalDesiredPose(targetState).getShoulderAngle()),
                Units.radiansToDegrees(struct.getCurrentPose().getShoulderAngle()),
                tol)
            || score)
        && targetState != robotState.ALGAE_FLINGUP)) { // just
      if (targetState == robotState.L1_PRESCORE) {
        gripper.runVolts(() -> 7);
      } else if (targetState == robotState.PROCESSOR) {
        gripper.runVolts(() -> 12);
      } else {
        gripper.runVolts(() -> 4);
      }
      if (targetState != robotState.L1_PRESCORE
          && targetState != robotState.ALGAE_FLINGUP
          && targetState != robotState.L4_FINALE
          && targetState != robotState.PROCESSOR) {
        if (struct.isFront) {
          drive.runVelocity(
              ChassisSpeeds.fromRobotRelativeSpeeds(-2, 0, 0, Rotation2d.fromDegrees(0)));
        } else {
          drive.runVelocity(
              ChassisSpeeds.fromRobotRelativeSpeeds(2, 0, 0, Rotation2d.fromDegrees(0)));
        }
      }
    }
    // if (((MathUtil.isNear(
    //
    // Units.radiansToDegrees(struct.getOriginalDesiredPose(targetState).getShoulderAngle()),
    //         Units.radiansToDegrees(struct.getCurrentPose().getShoulderAngle()),
    //         tol))
    //     && targetState == robotState.ALGAE_FLINGUP)) { // just
    //   if (drive.getPose().getTranslation().getDistance(startingPose.getTranslation()) > 0.05)
    //     gripper.runVolts(() -> 12);

    //   if (struct.isFront) {
    //     drive.runVelocity(
    //         ChassisSpeeds.fromRobotRelativeSpeeds(2, 0, 0, Rotation2d.fromDegrees(0)));
    //   } else {
    //     drive.runVelocity(
    //         ChassisSpeeds.fromRobotRelativeSpeeds(-2, 0, 0, Rotation2d.fromDegrees(0)));
    //   }
    // }
    if (autoFinale
        && MathUtil.isNear(
            struct.getOriginalDesiredPose(targetState).getElevatorHeight(),
            struct.getCurrentPose().getElevatorHeight(),
            Units.inchesToMeters(3))) {
      gripper.runVolts(() -> 12);

      if (MathUtil.isNear(
          struct.getOriginalDesiredPose(targetState).getElevatorHeight(),
          struct.getCurrentPose().getElevatorHeight(),
          Units.inchesToMeters(1))) {
        if (struct.isFront) {
          drive.runVelocity(
              ChassisSpeeds.fromRobotRelativeSpeeds(-2, 0, 0, Rotation2d.fromDegrees(0)));
        } else {
          drive.runVelocity(
              ChassisSpeeds.fromRobotRelativeSpeeds(2, 0, 0, Rotation2d.fromDegrees(0)));
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // drive.stop();
    gripper.runVolts(() -> 0);
    gripper.setFling(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (targetState == robotState.ALGAE_FLINGUP) {
      return struct.isAtDesiredPose(RobotPoseDefinitions.ALGAE_FLINGUP);
    } else
      return drive.getPose().getTranslation().getDistance(startingPose.getTranslation()) > 0.05;
  }
}
