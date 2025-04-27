package frc.robot.commands;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.SuperStructure.Superstructure;
import frc.robot.subsystems.SuperStructure.Superstructure.robotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.util.MathUtils;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class AlignToNearestPoseCommand extends Command {
  private double prevPIDoutputX = -99;
  private double prevPIDoutputY = -99;
  private double prevTime = 0;

  private final Drive drive;
  private List<Pose2d> targetPoses;
  private Pose2d targetPose;
  private final List<Pose2d> redTargetPoses;
  private final List<Pose2d> blueTargetPoses;
  private final Map<Pose2d, Transform2d> leftOffsets = new HashMap<>();
  private final Map<Pose2d, Transform2d> rightOffsets = new HashMap<>();
  private final Map<Pose2d, Transform2d> centerOffsets = new HashMap<>();
  private Pose2d currentPose;
  PIDConstants xConstants = new PIDConstants(5, 0.0, 0.0);
  PIDConstants yConstants = new PIDConstants(5, 0.0, 0.0);
  PIDConstants thetaConstants = new PIDConstants(6., 0.0, 0.0);
  Boolean alignFirstTagAuto = false;
  Boolean alignFirstTagLeft = false;

  int setTagToAlignT = -1;
  TrapezoidProfile.Constraints xConstraints = new TrapezoidProfile.Constraints(.01, 3); // 200
  TrapezoidProfile.Constraints yConstraints = new TrapezoidProfile.Constraints(.01, 3);
  TrapezoidProfile.Constraints thetaConstraints = new TrapezoidProfile.Constraints(10.0, 20.0);
  private final ProfiledPIDController xController =
      new ProfiledPIDController(xConstants.kP, xConstants.kI, xConstants.kD, xConstraints);
  private final ProfiledPIDController yController =
      new ProfiledPIDController(yConstants.kP, yConstants.kI, yConstants.kD, yConstraints);
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          thetaConstants.kP, thetaConstants.kI, thetaConstants.kD, thetaConstraints);
  private Pose2d relativePose;

  private final double ACCEL_CONSTRAINT_X = 7;
  private final double ACCEL_CONSTRAINT_Y = 7;

  public enum AlignmentDirection {
    LEFT,
    CENTER,
    RIGHT,
    HP,
    PROCESSOR,
    NET,
    CORAL
  };

  @AutoLogOutput private AlignmentDirection direction;
  private AlignmentDirection localDirection;

  private final AprilTagFieldLayout layout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  private void createOffsets() {
    switch (Constants.currentField) {
      case N03:
        leftOffsets.put(
            layout.getTagPose(6).get().toPose2d(),
            new Transform2d(.42, -0.126, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(7).get().toPose2d(),
            new Transform2d(.461, -0.131, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(8).get().toPose2d(),
            new Transform2d(.45, -0.112, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(9).get().toPose2d(),
            new Transform2d(.45, -0.147, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(10).get().toPose2d(),
            new Transform2d(.413, -0.181, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(11).get().toPose2d(),
            new Transform2d(.42, -0.126, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(17).get().toPose2d(),
            new Transform2d(.42, -0.126, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(18).get().toPose2d(),
            new Transform2d(.42, -0.126, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(19).get().toPose2d(),
            new Transform2d(.42, -0.126, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(20).get().toPose2d(),
            new Transform2d(.42, -0.126, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(21).get().toPose2d(),
            new Transform2d(.42, -0.126, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(22).get().toPose2d(),
            new Transform2d(.42, -0.126, new Rotation2d(Math.PI)));

        rightOffsets.put(
            layout.getTagPose(6).get().toPose2d(),
            new Transform2d(.42, .263, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(7).get().toPose2d(),
            new Transform2d(.461, .223, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(8).get().toPose2d(),
            new Transform2d(.44 + Units.inchesToMeters(0.5), .239, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(9).get().toPose2d(),
            new Transform2d(.45, .204, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(10).get().toPose2d(),
            new Transform2d(.408, .156, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(11).get().toPose2d(),
            new Transform2d(.42, .263, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(17).get().toPose2d(),
            new Transform2d(.42, .263, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(18).get().toPose2d(),
            new Transform2d(.42, .263, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(19).get().toPose2d(),
            new Transform2d(.42, .263, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(20).get().toPose2d(),
            new Transform2d(.42, .263, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(21).get().toPose2d(),
            new Transform2d(.42, .263, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(22).get().toPose2d(),
            new Transform2d(.42, .263, new Rotation2d(Math.PI)));

        centerOffsets.put(
            layout.getTagPose(6).get().toPose2d(),
            new Transform2d(.461, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(7).get().toPose2d(),
            new Transform2d(.461, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(8).get().toPose2d(),
            new Transform2d(.461, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(9).get().toPose2d(),
            new Transform2d(.461, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(10).get().toPose2d(),
            new Transform2d(.413 - Units.inchesToMeters(1), 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(11).get().toPose2d(),
            new Transform2d(.461, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(17).get().toPose2d(),
            new Transform2d(.461, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(18).get().toPose2d(),
            new Transform2d(.461, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(19).get().toPose2d(),
            new Transform2d(.461, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(20).get().toPose2d(),
            new Transform2d(.461, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(21).get().toPose2d(),
            new Transform2d(.461, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(22).get().toPose2d(),
            new Transform2d(.461, 0, new Rotation2d(Math.PI)));
        break;
      case CYBERSONICS:
        leftOffsets.put(
            layout.getTagPose(6).get().toPose2d(), new Transform2d(0, 0, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(7).get().toPose2d(), new Transform2d(0, 0, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(8).get().toPose2d(), new Transform2d(0, 0, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(9).get().toPose2d(), new Transform2d(0, 0, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(10).get().toPose2d(), new Transform2d(0, 0, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(11).get().toPose2d(), new Transform2d(0, 0, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(17).get().toPose2d(),
            new Transform2d(0.425, -0.105, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(18).get().toPose2d(),
            new Transform2d(.439, -0.092, new Rotation2d(Math.PI)));

        leftOffsets.put(
            layout.getTagPose(19).get().toPose2d(),
            new Transform2d(.432, -0.108, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(20).get().toPose2d(),
            new Transform2d(.431, -0.1, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(21).get().toPose2d(),
            new Transform2d(0.437, -0.109, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(22).get().toPose2d(),
            new Transform2d(0.435, -0.095, new Rotation2d(Math.PI)));

        rightOffsets.put(
            layout.getTagPose(6).get().toPose2d(), new Transform2d(0, 0, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(7).get().toPose2d(), new Transform2d(0, 0, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(8).get().toPose2d(), new Transform2d(0, 0, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(9).get().toPose2d(), new Transform2d(0, 0, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(10).get().toPose2d(), new Transform2d(0, 0, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(11).get().toPose2d(), new Transform2d(0, 0, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(17).get().toPose2d(),
            new Transform2d(0.428, 0.238, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(18).get().toPose2d(),
            new Transform2d(.443, .222, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(19).get().toPose2d(),
            new Transform2d(.439, .229, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(20).get().toPose2d(),
            new Transform2d(.433, .198, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(21).get().toPose2d(),
            new Transform2d(0.452, 0.216, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(22).get().toPose2d(),
            new Transform2d(0.405, 0.272, new Rotation2d(Math.PI)));

        centerOffsets.put(
            layout.getTagPose(6).get().toPose2d(), new Transform2d(0, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(7).get().toPose2d(), new Transform2d(0, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(8).get().toPose2d(), new Transform2d(0, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(9).get().toPose2d(), new Transform2d(0, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(10).get().toPose2d(), new Transform2d(0, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(11).get().toPose2d(), new Transform2d(0, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(17).get().toPose2d(), new Transform2d(0, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(18).get().toPose2d(), new Transform2d(0, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(19).get().toPose2d(), new Transform2d(0, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(20).get().toPose2d(), new Transform2d(0, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(21).get().toPose2d(), new Transform2d(0, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(22).get().toPose2d(),
            new Transform2d(0.442, 0.172, new Rotation2d(Math.PI)));
        break;

      case NORMAL:
        leftOffsets.put(
            layout.getTagPose(6).get().toPose2d(),
            new Transform2d(0.434, -.147, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(7).get().toPose2d(),
            new Transform2d(0.427, -.137, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(8).get().toPose2d(),
            new Transform2d(0.437, -.132, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(9).get().toPose2d(),
            new Transform2d(0.413, -.103, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(10).get().toPose2d(),
            new Transform2d(0.438, -.105, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(11).get().toPose2d(),
            new Transform2d(0.435, -.147, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(17).get().toPose2d(),
            new Transform2d(0.413, -.103, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(18).get().toPose2d(),
            new Transform2d(0.413, -.103, new Rotation2d(Math.PI)));

        leftOffsets.put(
            layout.getTagPose(19).get().toPose2d(),
            new Transform2d(0.413, -.103, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(20).get().toPose2d(),
            new Transform2d(0.413, -.103, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(21).get().toPose2d(),
            new Transform2d(0.413, -.103, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(22).get().toPose2d(),
            new Transform2d(0.413, -.103, new Rotation2d(Math.PI)));

        rightOffsets.put(
            layout.getTagPose(6).get().toPose2d(),
            new Transform2d(.43, .224, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(7).get().toPose2d(),
            new Transform2d(.441, .218, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(8).get().toPose2d(),
            new Transform2d(.433, .179, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(9).get().toPose2d(),
            new Transform2d(.417, .229, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(10).get().toPose2d(),
            new Transform2d(.424, .249, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(11).get().toPose2d(),
            new Transform2d(.42, .233, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(17).get().toPose2d(),
            new Transform2d(.417, .229, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(18).get().toPose2d(),
            new Transform2d(.417, .229, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(19).get().toPose2d(),
            new Transform2d(.417, .229, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(20).get().toPose2d(),
            new Transform2d(.417, .229, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(21).get().toPose2d(),
            new Transform2d(.417, .229, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(22).get().toPose2d(),
            new Transform2d(.417, .229, new Rotation2d(Math.PI)));

        centerOffsets.put(
            layout.getTagPose(6).get().toPose2d(), new Transform2d(.8, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(7).get().toPose2d(),
            new Transform2d(0.8, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(8).get().toPose2d(),
            new Transform2d(0.8, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(9).get().toPose2d(),
            new Transform2d(0.8, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(10).get().toPose2d(),
            new Transform2d(0.8, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(11).get().toPose2d(),
            new Transform2d(.8, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(17).get().toPose2d(),
            new Transform2d(.8, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(18).get().toPose2d(),
            new Transform2d(0.8, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(19).get().toPose2d(),
            new Transform2d(0.8, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(20).get().toPose2d(),
            new Transform2d(0.8, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(21).get().toPose2d(),
            new Transform2d(0.8, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(22).get().toPose2d(),
            new Transform2d(0.8, 0, new Rotation2d(Math.PI)));
        break;
      case HH:
      case SCH:
        leftOffsets.put(
            layout.getTagPose(6).get().toPose2d(),
            new Transform2d(0.42, -.125, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(7).get().toPose2d(),
            new Transform2d(0.415, -.125, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(8).get().toPose2d(),
            new Transform2d(0.425, -.117, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(9).get().toPose2d(),
            new Transform2d(0.423, -.125, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(10).get().toPose2d(),
            new Transform2d(0.415, -.125, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(11).get().toPose2d(),
            new Transform2d(0.432, -.125, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(17).get().toPose2d(),
            new Transform2d(0.421, -.129, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(18).get().toPose2d(),
            new Transform2d(0.415, -.125, new Rotation2d(Math.PI)));

        leftOffsets.put(
            layout.getTagPose(19).get().toPose2d(),
            new Transform2d(0.422, -.120, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(20).get().toPose2d(),
            new Transform2d(0.422, -.121, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(21).get().toPose2d(),
            new Transform2d(0.415, -.125, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(22).get().toPose2d(),
            new Transform2d(0.424, -.11, new Rotation2d(Math.PI)));

        rightOffsets.put(
            layout.getTagPose(6).get().toPose2d(),
            new Transform2d(.421, .222, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(7).get().toPose2d(),
            new Transform2d(.415, .215, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(8).get().toPose2d(),
            new Transform2d(.423, .198, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(9).get().toPose2d(),
            new Transform2d(.42, .228, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(10).get().toPose2d(),
            new Transform2d(.415, .215, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(11).get().toPose2d(),
            new Transform2d(.421, .227, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(17).get().toPose2d(),
            new Transform2d(.421, .191, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(18).get().toPose2d(),
            new Transform2d(.415, .215, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(19).get().toPose2d(),
            new Transform2d(.418, .21, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(20).get().toPose2d(),
            new Transform2d(.419, .210, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(21).get().toPose2d(),
            new Transform2d(.415, .215, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(22).get().toPose2d(),
            new Transform2d(.417, .215, new Rotation2d(Math.PI)));

        centerOffsets.put(
            layout.getTagPose(6).get().toPose2d(),
            new Transform2d(0.408, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(7).get().toPose2d(),
            new Transform2d(0.408, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(8).get().toPose2d(),
            new Transform2d(0.408, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(9).get().toPose2d(),
            new Transform2d(0.408, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(10).get().toPose2d(),
            new Transform2d(0.408, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(11).get().toPose2d(),
            new Transform2d(0.408, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(17).get().toPose2d(),
            new Transform2d(0.408, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(18).get().toPose2d(),
            new Transform2d(0.408, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(19).get().toPose2d(),
            new Transform2d(0.408, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(20).get().toPose2d(),
            new Transform2d(0.408, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(21).get().toPose2d(),
            new Transform2d(0.408, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(22).get().toPose2d(),
            new Transform2d(0.408, 0, new Rotation2d(Math.PI)));
        break;

      case DCMP:
        leftOffsets.put(
            layout.getTagPose(6).get().toPose2d(),
            new Transform2d(0.44, -.125, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(7).get().toPose2d(),
            new Transform2d(0.44, -.13, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(8).get().toPose2d(),
            new Transform2d(0.44, -.117, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(9).get().toPose2d(),
            new Transform2d(0.443, -.125, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(10).get().toPose2d(),
            new Transform2d(0.435, -.125, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(11).get().toPose2d(),
            new Transform2d(0.442, -.135, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(17).get().toPose2d(),
            new Transform2d(0.441, -.124, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(18).get().toPose2d(),
            new Transform2d(0.4525, -.125, new Rotation2d(Math.PI)));

        leftOffsets.put(
            layout.getTagPose(19).get().toPose2d(),
            new Transform2d(0.442, -.120, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(20).get().toPose2d(),
            new Transform2d(0.442, -.121, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(21).get().toPose2d(),
            new Transform2d(0.44, -.125, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(22).get().toPose2d(),
            new Transform2d(0.444, -.11, new Rotation2d(Math.PI)));

        rightOffsets.put(
            layout.getTagPose(6).get().toPose2d(),
            new Transform2d(.445, .210, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(7).get().toPose2d(),
            new Transform2d(.45, .215, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(8).get().toPose2d(),
            new Transform2d(.443, .198, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(9).get().toPose2d(),
            new Transform2d(.44, .228, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(10).get().toPose2d(),
            new Transform2d(.44, .215, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(11).get().toPose2d(),
            new Transform2d(.441, .227, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(17).get().toPose2d(),
            new Transform2d(.441, .238, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(18).get().toPose2d(),
            new Transform2d(.435, .215, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(19).get().toPose2d(),
            new Transform2d(.438, .21, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(20).get().toPose2d(),
            new Transform2d(.439, .210, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(21).get().toPose2d(),
            new Transform2d(.44, .215, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(22).get().toPose2d(),
            new Transform2d(.445, .227, new Rotation2d(Math.PI)));

        centerOffsets.put(
            layout.getTagPose(6).get().toPose2d(),
            new Transform2d(0.461, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(7).get().toPose2d(),
            new Transform2d(0.461, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(8).get().toPose2d(),
            new Transform2d(0.461, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(9).get().toPose2d(),
            new Transform2d(0.461, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(10).get().toPose2d(),
            new Transform2d(0.461, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(11).get().toPose2d(),
            new Transform2d(0.461, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(17).get().toPose2d(),
            new Transform2d(0.461, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(18).get().toPose2d(),
            new Transform2d(0.461, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(19).get().toPose2d(),
            new Transform2d(0.461, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(20).get().toPose2d(),
            new Transform2d(0.461, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(21).get().toPose2d(),
            new Transform2d(0.461, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(22).get().toPose2d(),
            new Transform2d(0.461, 0, new Rotation2d(Math.PI)));
      case KRYPTON:
        leftOffsets.put(
            layout.getTagPose(6).get().toPose2d(),
            new Transform2d(.45, -0.112, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(7).get().toPose2d(),
            new Transform2d(.427, -0.126, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(8).get().toPose2d(),
            new Transform2d(.45, -0.112, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(9).get().toPose2d(),
            new Transform2d(.45, -0.112, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(10).get().toPose2d(),
            new Transform2d(.45, -0.112, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(11).get().toPose2d(),
            new Transform2d(.45, -0.112, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(17).get().toPose2d(),
            new Transform2d(.45, -0.112, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(18).get().toPose2d(),
            new Transform2d(.45, -0.112, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(19).get().toPose2d(),
            new Transform2d(.45, -0.112, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(20).get().toPose2d(),
            new Transform2d(.45, -0.112, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(21).get().toPose2d(),
            new Transform2d(.45, -0.112, new Rotation2d(Math.PI)));
        leftOffsets.put(
            layout.getTagPose(22).get().toPose2d(),
            new Transform2d(.45, -0.112, new Rotation2d(Math.PI)));

        rightOffsets.put(
            layout.getTagPose(6).get().toPose2d(),
            new Transform2d(.45, .239, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(7).get().toPose2d(),
            new Transform2d(.426, .232, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(8).get().toPose2d(),
            new Transform2d(.45, .232, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(9).get().toPose2d(),
            new Transform2d(.45, .239, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(10).get().toPose2d(),
            new Transform2d(.45, .239, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(11).get().toPose2d(),
            new Transform2d(.45, .239, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(17).get().toPose2d(),
            new Transform2d(.45, .239, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(18).get().toPose2d(),
            new Transform2d(.45, .239, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(19).get().toPose2d(),
            new Transform2d(.45, .239, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(20).get().toPose2d(),
            new Transform2d(.45, .239, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(21).get().toPose2d(),
            new Transform2d(.45, .239, new Rotation2d(Math.PI)));
        rightOffsets.put(
            layout.getTagPose(22).get().toPose2d(),
            new Transform2d(.45, .239, new Rotation2d(Math.PI)));

        centerOffsets.put(
            layout.getTagPose(6).get().toPose2d(),
            new Transform2d(.461, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(7).get().toPose2d(),
            new Transform2d(.461, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(8).get().toPose2d(),
            new Transform2d(.461, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(9).get().toPose2d(),
            new Transform2d(.461, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(10).get().toPose2d(),
            new Transform2d(.413 - Units.inchesToMeters(1), 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(11).get().toPose2d(),
            new Transform2d(.461, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(17).get().toPose2d(),
            new Transform2d(.461, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(18).get().toPose2d(),
            new Transform2d(.461, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(19).get().toPose2d(),
            new Transform2d(.461, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(20).get().toPose2d(),
            new Transform2d(.461, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(21).get().toPose2d(),
            new Transform2d(.461, 0, new Rotation2d(Math.PI)));
        centerOffsets.put(
            layout.getTagPose(22).get().toPose2d(),
            new Transform2d(.461, 0, new Rotation2d(Math.PI)));
        break;
    }
  }

  public AlignToNearestPoseCommand(
      Drive drive, AlignmentDirection alignmentDirection, boolean alignFirst, boolean isLeftFirst) {
    this(drive, alignmentDirection);

    this.alignFirstTagAuto = alignFirst;
    this.alignFirstTagLeft = isLeftFirst;
  }

  public AlignToNearestPoseCommand(Drive drive, AlignmentDirection alignmentDirection) {
    direction = alignmentDirection;
    createOffsets();
    this.drive = drive;
    this.alignFirstTagAuto = false;
    this.alignFirstTagLeft = false;

    // Initialize target poses for red and blue alliances
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

    // Initialize PID controllers with default values

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // Initialize NetworkTable entries
    // NetworkTable pidTable = NetworkTableInstance.getDefault().getTable("AlignPIDTuning");
    // xPEntry = pidTable.getEntry("xP");
    // xIEntry = pidTable.getEntry("xI");
    // xDEntry = pidTable.getEntry("xD");
    // xMaxVelEntry = pidTable.getEntry("xMaxVel");
    // xMaxAccelEntry = pidTable.getEntry("xMaxAccel");

    // yPEntry = pidTable.getEntry("yP");
    // yIEntry = pidTable.getEntry("yI");
    // yDEntry = pidTable.getEntry("yD");
    // yMaxVelEntry = pidTable.getEntry("yMaxVel");
    // yMaxAccelEntry = pidTable.getEntry("yMaxAccel");

    // thetaPEntry = pidTable.getEntry("thetaP");
    // thetaIEntry = pidTable.getEntry("thetaI");
    // thetaDEntry = pidTable.getEntry("thetaD");
    // thetaMaxVelEntry = pidTable.getEntry("thetaMaxVel");
    // thetaMaxAccelEntry = pidTable.getEntry("thetaMaxAccel");

    // // // NT Defaults
    // xPEntry.setDouble(xConstants.kP);
    // xIEntry.setDouble(xConstants.kI);
    // xDEntry.setDouble(xConstants.kD);
    // xMaxVelEntry.setDouble(xConstraints.maxVelocity);
    // xMaxAccelEntry.setDouble(xConstraints.maxAcceleration);

    // yPEntry.setDouble(yConstants.kP);
    // yIEntry.setDouble(yConstants.kI);
    // yDEntry.setDouble(yConstants.kD);
    // yMaxVelEntry.setDouble(yConstraints.maxVelocity);
    // yMaxAccelEntry.setDouble(yConstraints.maxAcceleration);

    // thetaPEntry.setDouble(thetaConstants.kP);
    // thetaIEntry.setDouble(thetaConstants.kI);
    // thetaDEntry.setDouble(thetaConstants.kD);
    // thetaMaxVelEntry.setDouble(thetaConstraints.maxVelocity);
    // thetaMaxAccelEntry.setDouble(thetaConstraints.maxAcceleration);

    addRequirements(drive);
  }

  @Override
  public void initialize() {

    localDirection = direction;
    // -----------------
    LED.aligning = true;
    LED.isAligned = false;
    // ------------

    if (Superstructure.isAlgae && direction == AlignmentDirection.HP) {
      localDirection = AlignmentDirection.PROCESSOR;
    }
    if (Superstructure.isAlgae && direction == AlignmentDirection.LEFT) {
      localDirection = AlignmentDirection.CENTER;
    }
    if (Superstructure.isAlgae && direction == AlignmentDirection.RIGHT) {
      localDirection = AlignmentDirection.NET;
    }

    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Blue) {
      targetPoses = blueTargetPoses;

      if (localDirection == AlignmentDirection.HP) {
        targetPoses =
            List.of(layout.getTagPose(12).get().toPose2d(), layout.getTagPose(13).get().toPose2d());
      } else if (localDirection == AlignmentDirection.PROCESSOR) {
        targetPoses = List.of(layout.getTagPose(16).get().toPose2d());
      } else if (localDirection == AlignmentDirection.NET) {
        targetPoses = List.of(new Pose2d(new Translation2d(7.020, 0), new Rotation2d()));
      }
    } else {
      targetPoses = redTargetPoses;
      if (localDirection == AlignmentDirection.HP) {
        targetPoses =
            List.of(layout.getTagPose(1).get().toPose2d(), layout.getTagPose(2).get().toPose2d());
      } else if (localDirection == AlignmentDirection.PROCESSOR) {
        targetPoses = List.of(layout.getTagPose(3).get().toPose2d());
      } else if (localDirection == AlignmentDirection.NET) {
        targetPoses = List.of(new Pose2d(new Translation2d(10.511, 0), new Rotation2d()));
      } else if (localDirection == AlignmentDirection.CORAL) {
        targetPoses =
            List.of(
                new Pose2d(16.325, 5.840, Rotation2d.fromDegrees(46.600)),
                new Pose2d(16.325, 4.025, Rotation2d.fromDegrees(180.000)));
      }
    }

    currentPose = drive.getPose();

    targetPose =
        targetPoses.stream()
            .min(
                Comparator.comparingDouble(
                    pose -> currentPose.getTranslation().getDistance(pose.getTranslation())))
            .orElseThrow();
    if (localDirection != AlignmentDirection.NET && localDirection != AlignmentDirection.CORAL) {
      for (int i = 1; i <= 22; i++) {
        if (layout.getTagPose(i).isPresent()
            && layout.getTagPose(i).get().toPose2d().equals(targetPose)) {
          Superstructure.alignedTag = i;
          break;
        }
      }
    }

    switch (localDirection) {
      case LEFT:
        targetPose = targetPose.plus(leftOffsets.get(targetPose));
        break;
      case RIGHT:
        targetPose = targetPose.plus(rightOffsets.get(targetPose));
        break;
      case CENTER:
        targetPose = targetPose.plus(centerOffsets.get(targetPose));
        break;
      case HP:
        targetPose =
            targetPose.plus(
                new Transform2d(Units.inchesToMeters(17.0), 0.06, Rotation2d.fromRadians(Math.PI)));
        break;
      case PROCESSOR:
        targetPose =
            targetPose.plus(
                new Transform2d(
                    Units.inchesToMeters(45),
                    Units.inchesToMeters(1.1),
                    Rotation2d.fromRadians(Math.PI)));
        break;
      case CORAL:
        break;
      default:
        break;
    }
    if (localDirection != direction.PROCESSOR) {
      if (Math.abs(
              MathUtils.boundAngleNegPiToPiRadians(
                  targetPose.getRotation().getRadians() - drive.getRotation().getRadians()))
          < Math.abs(
              MathUtils.boundAngleNegPiToPiRadians(
                  targetPose.getRotation().getRadians()
                      - Math.PI
                      - drive.getRotation().getRadians()))) {
        targetPose = targetPose;
        Superstructure.isFront = true;

      } else {
        Superstructure.isFront = false;

        targetPose =
            new Pose2d(
                targetPose.getX(),
                targetPose.getY(),
                Rotation2d.fromRadians(targetPose.getRotation().getRadians() - Math.PI));
      }
    } else {
      Superstructure.isFront = true;
    }

    if (alignFirstTagAuto) {

      if (DriverStation.getAlliance().isPresent()
          && DriverStation.getAlliance().get() == Alliance.Blue) {
        if (alignFirstTagLeft) {
          setTagToAlignT = 20;
        } else if (!alignFirstTagLeft) {
          setTagToAlignT = 22;
        }
      } else {
        if (alignFirstTagLeft) {
          setTagToAlignT = 11;
        } else if (!alignFirstTagLeft) {
          setTagToAlignT = 9;
        }
      }
      targetPose = layout.getTagPose(setTagToAlignT).get().toPose2d();
      Superstructure.isFront = true;

      switch (localDirection) {
        case LEFT:
          targetPose = targetPose.plus(leftOffsets.get(targetPose));
          break;
        case RIGHT:
          targetPose = targetPose.plus(rightOffsets.get(targetPose));
          break;
        case CENTER:
          targetPose = targetPose.plus(centerOffsets.get(targetPose));
      }
    }

    Logger.recordOutput("Align/TargetPose", targetPose);
    xController.setGoal(0);
    yController.setGoal(0);
    thetaController.setGoal(0);
    xController.setTolerance(Units.inchesToMeters(0.15)); // was 0.2 / 0.15
    yController.setTolerance(Units.inchesToMeters(0.15));
    thetaController.setTolerance(Units.degreesToRadians(1.5));

    if (direction.equals(AlignmentDirection.HP)) {
      xController.setTolerance(Units.inchesToMeters(Math.sqrt(3.0)));
      yController.setTolerance(Units.inchesToMeters(Math.sqrt(3.0)));
      thetaController.setTolerance(Units.degreesToRadians(1));
    }
    Superstructure.direction = localDirection;

    // .765, -58.385
  }

  @Override
  public void execute() {
    currentPose = drive.getPose();
    relativePose = drive.getPose().relativeTo(targetPose);

    double xSpeed = xController.calculate(relativePose.getX());
    double ySpeed = yController.calculate(relativePose.getY());

    double thetaSpeed = thetaController.calculate(relativePose.getRotation().getRadians());

    double time = Timer.getFPGATimestamp();

    if (prevTime == 0) {
      prevTime = time;
    }

    if (Math.abs(relativePose.getY()) > 0.30
        || Math.abs(relativePose.getRotation().getDegrees()) > 7.
            && localDirection != AlignmentDirection.CORAL) {
      if (localDirection != AlignmentDirection.NET) {
        xSpeed = 0;
      }
    }
    if (localDirection == AlignmentDirection.NET) {
      ySpeed = 0;
    }

    if (prevPIDoutputX == -99) {
      prevPIDoutputX = xSpeed;

    } else {
      xSpeed =
          prevPIDoutputX
              + MathUtil.clamp(
                  xSpeed - prevPIDoutputX,
                  -ACCEL_CONSTRAINT_X * (time - prevTime),
                  ACCEL_CONSTRAINT_X * (time - prevTime));
      prevPIDoutputX = xSpeed;
    }

    if (prevPIDoutputY == -99) {
      prevPIDoutputY = ySpeed;

    } else {
      ySpeed =
          prevPIDoutputY
              + MathUtil.clamp(
                  ySpeed - prevPIDoutputY,
                  -ACCEL_CONSTRAINT_Y * (time - prevTime),
                  ACCEL_CONSTRAINT_Y * (time - prevTime));
      prevPIDoutputY = ySpeed;
    }

    prevTime = time;
    double xClamp = 0.9;
    double yClamp = 1.75;

    if (Superstructure.getInstance().getCurrentState() == robotState.L4_PRESCORE) {
      xClamp = xClamp * 0.7;
      yClamp = yClamp * 0.7;
    }
    xSpeed = MathUtil.clamp(xSpeed, -xClamp, xClamp);
    ySpeed = MathUtil.clamp(ySpeed, -yClamp, yClamp);

    Translation2d targetFrameVel = new Translation2d(xSpeed, ySpeed);
    Translation2d fieldFrameVel = targetFrameVel.rotateBy(currentPose.getRotation());
    ChassisSpeeds robotSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldFrameVel.getX(), fieldFrameVel.getY(), thetaSpeed, currentPose.getRotation());

    drive.runVelocity(robotSpeeds);

    // Logger.recordOutput("sigmakp", xController.getP());

    Logger.recordOutput("Align/xSpeed", xSpeed);
    Logger.recordOutput("Align/ySpeed", ySpeed);
    // Logger.recordOutput("Align/rotated speed x", fiedFrameVel.getX());
    // Logger.recordOutput("Align/rotated speed y", fiedFrameVel.getY());
  }

  //   private void updatePIDValues() {
  //     // x
  //     xController.setP(xPEntry.getDouble(xController.getP()));
  //     xController.setI(xIEntry.getDouble(xController.getI()));
  //     xController.setD(xDEntry.getDouble(xController.getD()));
  //     xController.setConstraints(
  //         new TrapezoidProfile.Constraints(
  //             xMaxVelEntry.getDouble(xController.getConstraints().maxVelocity),
  //             xMaxAccelEntry.getDouble(xController.getConstraints().maxAcceleration)));

  //     // y
  //     yController.setP(yPEntry.getDouble(yController.getP()));
  //     yController.setI(yIEntry.getDouble(yController.getI()));
  //     yController.setD(yDEntry.getDouble(yController.getD()));
  //     yController.setConstraints(
  //         new TrapezoidProfile.Constraints(
  //             yMaxVelEntry.getDouble(yController.getConstraints().maxVelocity),
  //             yMaxAccelEntry.getDouble(yController.getConstraints().maxAcceleration)));

  //     // theta
  //     thetaController.setP(thetaPEntry.getDouble(thetaController.getP()));
  //     thetaController.setI(thetaIEntry.getDouble(thetaController.getI()));
  //     thetaController.setD(thetaDEntry.getDouble(thetaController.getD()));
  //     thetaController.setConstraints(
  //         new TrapezoidProfile.Constraints(
  //             thetaMaxVelEntry.getDouble(thetaController.getConstraints().maxVelocity),
  //             thetaMaxAccelEntry.getDouble(thetaController.getConstraints().maxAcceleration)));
  //   }

  @Override
  public void end(boolean interrupted) {
    LED.isAligned = true;
    LED.aligning = false;

    drive.stop();
    if (localDirection == AlignmentDirection.HP
        && Gripper.getInstance(Shoulder.getInstance().getAngleSupplier())
            .hasPiece()
            .getAsBoolean()) {
      Superstructure.isFront = !Superstructure.isFront;
    }
  }

  @Override
  public boolean isFinished() {
    return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
  }
}
