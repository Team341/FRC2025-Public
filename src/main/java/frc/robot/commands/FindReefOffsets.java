// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.Comparator;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FindReefOffsets extends Command {
  private final List<Pose2d> redTargetPoses;
  private final List<Pose2d> blueTargetPoses;
  private List<Pose2d> targetPoses;
  private final AprilTagFieldLayout layout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  private Pose2d targetPose;
  private Pose2d currentPose;
  private Drive drive;

  /** Creates a new FindReefOffsets. */
  public FindReefOffsets(Drive drive) {
    this.drive = drive;
    addRequirements(drive);
    redTargetPoses =
        List.of(
            layout.getTagPose(6).get().toPose2d(),
            layout.getTagPose(7).get().toPose2d(),
            layout.getTagPose(8).get().toPose2d(),
            layout.getTagPose(9).get().toPose2d(),
            layout.getTagPose(10).get().toPose2d(),
            layout.getTagPose(11).get().toPose2d());

    blueTargetPoses =
        List.of(
            layout.getTagPose(17).get().toPose2d(),
            layout.getTagPose(18).get().toPose2d(),
            layout.getTagPose(19).get().toPose2d(),
            layout.getTagPose(20).get().toPose2d(),
            layout.getTagPose(21).get().toPose2d(),
            layout.getTagPose(22).get().toPose2d());
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Blue) {
      targetPoses = blueTargetPoses;
    } else {
      targetPoses = redTargetPoses;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    currentPose = drive.getPose();
    targetPose =
        targetPoses.stream()
            .min(
                Comparator.comparingDouble(
                    pose -> currentPose.getTranslation().getDistance(pose.getTranslation())))
            .orElseThrow();
    Logger.recordOutput("ReefOffset Translation 2d", targetPose.minus(currentPose));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println(
        "Offset X "
            + targetPose.minus(currentPose).getX()
            + " Offset Y "
            + targetPose.minus(currentPose).getY());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
