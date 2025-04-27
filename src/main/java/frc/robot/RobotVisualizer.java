package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import java.util.function.DoubleSupplier;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class RobotVisualizer {
  private static DoubleSupplier elevatorHeight;
  private static DoubleSupplier shoulderAngle;
  private static DoubleSupplier wristAngle;
  private static DoubleSupplier coralAngle;
  @Setter private static boolean hasAlgae;

  public static void update() {
    if (shoulderAngle != null
        && elevatorHeight != null
        && wristAngle != null
        && coralAngle != null) {
      Logger.recordOutput(
          "RobotVisualizer/Elevator",
          new Pose3d(0, 0, elevatorHeight.getAsDouble(), new Rotation3d()));
      Logger.recordOutput(
          "RobotVisualizer/Shoulder",
          new Pose3d(
              0,
              0,
              elevatorHeight.getAsDouble() + 0.32,
              new Rotation3d(0, (shoulderAngle.getAsDouble()), 0)));
      Pose3d wristPose =
          new Pose3d(
              Units.inchesToMeters(7.) * Math.sin((shoulderAngle.getAsDouble())),
              0,
              elevatorHeight.getAsDouble()
                  + 0.32
                  + Units.inchesToMeters(7.) * Math.cos((shoulderAngle.getAsDouble())),
              (new Rotation3d(0, shoulderAngle.getAsDouble(), 0))
                  .rotateBy(
                      new Rotation3d(
                          VecBuilder.fill(
                              Math.sin(shoulderAngle.getAsDouble()),
                              0,
                              Math.cos(shoulderAngle.getAsDouble())),
                          wristAngle.getAsDouble())));

      Logger.recordOutput("RobotVisualizer/Wrist", wristPose);

      Logger.recordOutput(
          "RobotVisualizer/Coral",
          new Pose3d(-0.235, 0, 0.15, new Rotation3d(0, coralAngle.getAsDouble(), 0)));
      if (hasAlgae) {
        Logger.recordOutput("RobotVisualizer/AlgaePiece", wristPose);
        Logger.recordOutput(
            "RobotVisualizer/CoralPice", new Pose3d(1000000, 0, 0, new Rotation3d()));
      } else {
        Logger.recordOutput(
            "RobotVisualizer/AlgaePiece", new Pose3d(100000, 0, 0, new Rotation3d()));
        Logger.recordOutput("RobotVisualizer/CoralPiece", wristPose);
      }
    }
  }

  public static void addElevatorHeight(DoubleSupplier mElevatorHeight) {
    elevatorHeight = mElevatorHeight;
  }

  public static void addShoulderAngle(DoubleSupplier mShoulderAngle) {
    shoulderAngle = () -> -mShoulderAngle.getAsDouble();
  }

  public static void addWristAngle(DoubleSupplier mWristAngle) {
    wristAngle = mWristAngle;
  }

  public static void addCoralAngle(DoubleSupplier mCoralAngle) {
    coralAngle = mCoralAngle;
  }
}
