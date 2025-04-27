// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SuperStructure;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class RobotPoseDefinitions {

  public static final RobotPose GROUND_INTAKE =
      new RobotPose(
          0.2544,
          Units.degreesToRadians(126.5),
          Units.degreesToRadians(0),
          Units.degreesToRadians(0),
          1.,
          -1.);
  public static final RobotPose PLAYER_INTAKE =
      new RobotPose(
          0.19 + Units.inchesToMeters(1.) + Units.inchesToMeters(1),
          -Units.degreesToRadians(-38),
          0);

  public static final RobotPose L4_PRESCORE =
      new RobotPose(
          1.18 + Units.inchesToMeters(0.75) + Units.inchesToMeters(1),
          -Units.degreesToRadians(42),
          -Math.PI / 2.,
          Math.PI / 2.);

  public static final RobotPose L4_SCORE =
      new RobotPose(
          1.18 + Units.inchesToMeters(1), -Units.degreesToRadians(67), -Math.PI / 2., Math.PI / 2.);
  public static final RobotPose L3_PRESCORE =
      new RobotPose(
          0.7 - Units.inchesToMeters(1), -Units.degreesToRadians(40.), -Math.PI / 2., Math.PI / 2.);

  public static final RobotPose L3_SCORE =
      new RobotPose(.63, -Units.degreesToRadians(73), -Math.PI / 2., Math.PI / 2.);

  public static final RobotPose L2_PRESCORE =
      new RobotPose(0.316, -Units.degreesToRadians(43), -Math.PI / 2., Math.PI / 2.);
  public static final RobotPose L2_SCORE =
      new RobotPose(
          0.316 - Units.inchesToMeters(1),
          -Units.degreesToRadians(73),
          -Math.PI / 2.,
          Math.PI / 2.);

  public static final RobotPose L1_PRESCORE =
      new RobotPose(0.0, -Units.degreesToRadians(55), 0, Math.PI / 2.);

  public static final RobotPose L1_SCORE =
      new RobotPose(0.0, -Units.degreesToRadians(55), 0, Math.PI / 2.);

  public static final RobotPose L1_DOWNWARD =
      new RobotPose(0.412, -Units.degreesToRadians(116), 0, Math.PI / 2.);

  public static final RobotPose STOW =
      new RobotPose(0.02, -Units.degreesToRadians(0.), -Math.PI / 2., Math.PI / 2.); // 0.02 before
  public static final RobotPose INITIAL_STOW =
      new RobotPose(0.288, -Units.degreesToRadians(0.), -Math.PI / 2., Math.PI / 2.);
  public static final RobotPose STOW_FOR_ALGAE_AUTO =
      new RobotPose(0.02, -Units.degreesToRadians(0.), 0, Math.PI / 2.);
  public static final RobotPose VERTICAL_INTAKE =
      new RobotPose(0.0, -Units.degreesToRadians(98.5), Math.PI / 2., Math.PI / 2., 0, -1);
  public static final RobotPose L2_ALGAE =
      new RobotPose(0.65, -Units.degreesToRadians(100), Units.degreesToRadians(90), Math.PI / 2);
  public static final RobotPose L3_ALGAE =
      new RobotPose(1.04, -Units.degreesToRadians(100), Units.degreesToRadians(90), Math.PI / 2);
  public static final RobotPose CLIMB =
      new RobotPose(Units.inchesToMeters(4.), -Units.degreesToRadians(-80), -Math.PI / 2, 0);
  public static final RobotPose CLIMB_POST =
      new RobotPose(Units.inchesToMeters(0.), -Units.degreesToRadians(-90), -Math.PI / 2, 0);
  public static final RobotPose ALGAE_HOLDING = new RobotPose(0.4, 0, Math.PI / 2., Math.PI / 2.);

  public static final RobotPose ALGAE_FLING =
      new RobotPose(.5, Units.degreesToRadians(-130), 0, Math.PI / 2.)
          .setShoulderConfigs(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(1)
                  .withMotionMagicAcceleration(6));
  // end pose
  public static final RobotPose ALGAE_FLINGUP =
      new RobotPose(1.2 + Units.inchesToMeters(1), Units.degreesToRadians(20), 0, Math.PI / 2.)
          .setShoulderConfigs(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(0.55)
                  .withMotionMagicAcceleration(9))
          .setEleavtorConfigs(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(40)
                  .withMotionMagicAcceleration(185)); // moving elevator
  // midpoint
  public static final RobotPose ALGAE_FLINGUP_NEAR_TOP =
      new RobotPose(1.2, -Units.degreesToRadians(0), 0, Math.PI / 2.);

  public static final RobotPose ALGAE_GROUND_INTAKE =
      new RobotPose(0.101, -Units.degreesToRadians(112.0), 0, Units.degreesToRadians(90));
  public static final RobotPose PROCESSOR =
      new RobotPose(0.085, -Units.degreesToRadians(85), 0, Units.degreesToRadians(90))
          .setShoulderConfigs(
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(2)
                  .withMotionMagicCruiseVelocity(1));
  public static final RobotPose ALGAE_UPRIGHT_INTAKE =
      new RobotPose(0.25, -Units.degreesToRadians(101.93), 0, Units.degreesToRadians(90));
  public static final RobotPose BARGE = new RobotPose(0, 0, 0, 0);

  public static final RobotPose L4_PRESCORE_ZERO =
      new RobotPose(L4_PRESCORE.getElevatorHeight(), 0, L4_PRESCORE.getWristAngle(), Math.PI / 2.);
  public static final RobotPose L3_PRESCORE_ZERO =
      new RobotPose(L3_PRESCORE.getElevatorHeight(), 0, L3_PRESCORE.getWristAngle(), Math.PI / 2.);
  public static final RobotPose L2_PRESCORE_ZERO =
      new RobotPose(L2_PRESCORE.getElevatorHeight(), 0, L2_PRESCORE.getWristAngle(), Math.PI / 2.);

  public static final RobotPose L1_INVERTED =
      new RobotPose(0.388, Units.degreesToRadians(101.5), 0, Math.PI / 2.);

  public static final RobotPose L4_AUTO_FINALE =
      new RobotPose(0.74, Units.degreesToRadians(-58.3), -Math.PI / 2., Math.PI / 2);
}
