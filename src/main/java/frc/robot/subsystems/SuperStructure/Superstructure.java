// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SuperStructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlignToNearestPoseCommand.AlignmentDirection;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.coral.CoralConstants;
import frc.robot.subsystems.shoulder.ShoulderConstants;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

// What is about to follow is some of the most unreadable code of all time.
// This is a pretty stupid way of doing things but it works really well
// and we are #20 EPA in the world so idc.
// Please don't replicate this.
public class Superstructure {
  // good variables
  public final double groundToBumper = Units.inchesToMeters(1.5);
  public final double minimumElevatorBypassFromBottomHeight = 0;
  @Setter private double lastShoulderSign = 1;

  @AutoLogOutput public robotState currentState = robotState.INITIAL_STOW;
  public robotState desiredState = robotState.INITIAL_STOW;
  @Getter @Setter public RobotPose currentPose;
  @Setter private boolean isFloppy = false;
  @Getter private Trigger floppy = new Trigger(() -> isFloppy);
  @Getter public RobotPose currentDesiredPose;
  @AutoLogOutput public static boolean isFront = true;
  public static BooleanSupplier hasPiece;
  public static boolean isFlipped = false;
  public static boolean firstFlippedPose =
      false; // check to go back if we're flipping from a set posiitoin
  public static boolean climbing = false;
  public static int alignedTag = 0;
  @AutoLogOutput @Getter @Setter private boolean shoulderVertical = false;

  @AutoLogOutput @Getter @Setter public static boolean isAlgae = false;

  @Getter @Setter public static AlignmentDirection direction = AlignmentDirection.LEFT;

  public enum robotState {
    GROUND_INTAKE,
    PLAYER_INTAKE,
    L2_ALGAE_GRAB,
    L3_ALGAE_GRAB,
    L4_PRESCORE,
    L4_SCORE,
    L3_PRESCORE,
    L3_SCORE,
    L2_PRESCORE,
    L2_SCORE,
    L1_PRESCORE,
    L1_SCORE,
    L1_INVERTED,
    STOW,

    INITIAL_STOW,

    CLIMB,
    CLIMB_POST,
    PROCESSOR,
    L1_DOWNWARD,
    ALGAE_HOLDING,
    ALGAE_FLING,
    ALGAE_FLINGUP,
    BARGE,
    ALGAE_GROUND_INTAKE,
    ALGAE_UPRIGHT_INTAKE,
    STOW_FOR_ALGAE_AUTO,
    L4_PRESCORE_ZERO,
    L3_PRESCORE_ZERO,
    L2_PRESCORE_ZERO,
    L4_FINALE,

    VERTICAL_INTAKE
  }

  public robotState getCurrentState() {
    return currentState;
  }

  /** Creates a new Superstructure. */
  public Superstructure() {}

  private static Superstructure instance;

  public static Superstructure getInstance() {
    if (instance == null) {
      instance = new Superstructure();
      return instance;
    } else return instance;
  }

  // assumes everything is returning radians - which they should -> io should be
  // degrees and that
  // should return radians
  public RobotPose poseAdjust(RobotPose desiredPose, RobotPose currentPose) {
    this.currentPose = currentPose;
    RobotPose poseToReturn =
        new RobotPose(
                desiredPose.getElevatorHeight(),
                desiredPose.getShoulderAngle(),
                desiredPose.getWristAngle(),
                desiredPose.getCoralIntakeAngle(),
                desiredPose.getCoralIntakeSpeed(),
                desiredPose.getGripperSpeed())
            .setShoulderConfigs(desiredPose.getShoulderConfigs())
            .setEleavtorConfigs(desiredPose.getElevatorConfigs());
    if (isAlgae
        && (desiredState == robotState.L4_PRESCORE
            || desiredState == robotState.L4_SCORE
            || desiredState == robotState.L3_SCORE
            || desiredState == robotState.L3_PRESCORE
            || desiredState == robotState.L2_PRESCORE
            || desiredState == robotState.L1_PRESCORE
            || desiredState == robotState.L2_PRESCORE)) {
      if (alignedTag == 7
          || alignedTag == 9
          || alignedTag == 11
          || alignedTag == 20
          || alignedTag == 22
          || alignedTag == 18) {
        desiredState = robotState.L3_ALGAE_GRAB;
      }
      if (alignedTag == 6
          || alignedTag == 8
          || alignedTag == 10
          || alignedTag == 21
          || alignedTag == 17
          || alignedTag == 19) {
        desiredState = robotState.L2_ALGAE_GRAB;
      }
    }
    if (!isFront) {
      if (desiredState != robotState.GROUND_INTAKE
          && desiredState != robotState.ALGAE_HOLDING
          && desiredState != robotState.STOW
          && desiredState != robotState.INITIAL_STOW
          && desiredState != robotState.PLAYER_INTAKE
          && desiredState != robotState.ALGAE_GROUND_INTAKE
          && desiredState != robotState.ALGAE_UPRIGHT_INTAKE
          && desiredState != robotState.PROCESSOR
          && desiredState != robotState.CLIMB
          && desiredState != robotState.CLIMB_POST
          && desiredState
              != robotState.VERTICAL_INTAKE) { // TODO: make sure autos account for this.

        poseToReturn.setShoulderAngle(-poseToReturn.getShoulderAngle());
      }

      if (desiredState == robotState.PLAYER_INTAKE) {
        poseToReturn.setShoulderAngle(poseToReturn.getShoulderAngle());
      }
    }

    if (firstFlippedPose) {
      if (desiredState == robotState.L2_PRESCORE
          || desiredState == robotState.L2_SCORE
          || desiredState == robotState.L3_PRESCORE
          || desiredState == robotState.L3_SCORE
          || desiredState == robotState.L4_PRESCORE
          || desiredState == robotState.L4_SCORE) {
        if (desiredState == robotState.L3_PRESCORE
            || desiredState == robotState.L3_SCORE
            || desiredState == robotState.L4_PRESCORE
            || desiredState == robotState.L4_SCORE) {
          poseToReturn.setShoulderAngle(0);
        } else {
          poseToReturn.setShoulderAngle(-poseToReturn.getShoulderAngle());
        }
      }
    }
    if (!isAlgae && !isFront && desiredState == robotState.L2_SCORE) {
      poseToReturn.setCoralIntakeAngle(Units.degreesToRadians(72));
    }
    if (!isAlgae
        && !isFront
        && (desiredState == robotState.L1_PRESCORE || desiredState == robotState.L1_SCORE)) {

      desiredPose = RobotPoseDefinitions.L1_INVERTED;
      poseToReturn.setShoulderAngle(RobotPoseDefinitions.L1_INVERTED.getShoulderAngle());
      poseToReturn.setElevatorHeight(RobotPoseDefinitions.L1_INVERTED.getElevatorHeight());
      poseToReturn.setWristAngle(RobotPoseDefinitions.L1_INVERTED.getWristAngle());
    }

    if (climbing && !LED.climbed) {
      if (poseToReturn.getElevatorHeight() < Units.inchesToMeters(4)) {
        poseToReturn.setElevatorHeight(Units.inchesToMeters(4.));
      }
    }

    Logger.recordOutput(
        "SuperStructure/desired elevator precheck", poseToReturn.getElevatorHeight());
    Logger.recordOutput(
        "SuperStructure/desired shoulder precheck", poseToReturn.getShoulderAngle());
    Logger.recordOutput("SuperStructure/desired wrist precheck", poseToReturn.getWristAngle());
    Logger.recordOutput(
        "SuperStructure/desired coral precheck", poseToReturn.getCoralIntakeAngle());

    double shoulderZ =
        groundToBumper
            + ShoulderConstants.SHOULDER_LENGTH
                * Math.cos(Math.PI / 2. - (desiredPose.getShoulderAngle()));
    double shoulderX =
        desiredPose.getElevatorHeight()
            + ShoulderConstants.SHOULDER_LENGTH
                * Math.sin(Math.PI / 2. - (desiredPose.getShoulderAngle()));

    double coralZ =
        groundToBumper
            + Math.sin((currentPose.getCoralIntakeAngle())) * CoralConstants.APPROXIMATE_LENGTH;
    Logger.recordOutput("SuperStructure/CoralArmChecks/shoulder x", shoulderX);
    Logger.recordOutput("SuperStructure/CoralArmChecks/shoulder Z", shoulderZ);
    Logger.recordOutput("SuperStructure/CoralArmChecks/coral  Z", coralZ);

    if (shoulderZ < coralZ && shoulderX < CoralConstants.APPROXIMATE_X_ORIGIN) {
      double armThetaInCoralFrame =
          Math.acos(
              (shoulderX - CoralConstants.APPROXIMATE_X_ORIGIN)
                  / CoralConstants.APPROXIMATE_LENGTH);

      Logger.recordOutput(
          "SuperStructure/CoralArmChecks/arm in coral frame ", armThetaInCoralFrame);
    }

    if (Math.abs(desiredPose.getShoulderAngle()) < Math.abs(currentPose.getShoulderAngle())
        && Math.abs(
                Math.abs(currentPose.getShoulderAngle()) - Math.abs(desiredPose.getShoulderAngle()))
            > Units.degreesToRadians(70)) { // 35.
      poseToReturn.setCoralIntakeAngle(currentPose.getCoralIntakeAngle());
      if (currentPose.getCoralIntakeAngle() < Units.degreesToRadians(10)) {
        poseToReturn.setElevatorHeight(currentPose.getElevatorHeight());
      }
    }

    if (!isAlgae
        && Math.abs(desiredPose.getShoulderAngle()) < Math.abs(currentPose.getShoulderAngle())
        && Math.abs(currentPose.getShoulderAngle()) > Units.degreesToRadians(75.)) {
      poseToReturn.setWristAngle(currentPose.getWristAngle());
    }

    // ACTIVE COLLISION CHECKS

    // make sure that wrist angle is 0 by the time we whack into the elevator
    if (hasPiece.getAsBoolean()) {
      if (Math.abs(currentPose.getShoulderAngle()) < Units.degreesToRadians(30)
          && Math.abs(currentPose.getWristAngle()) - (Math.PI / 2.) > Units.degreesToRadians(3.0)) {
        if (!isAlgae) {
          poseToReturn.setShoulderAngle(currentPose.getShoulderAngle());
        }

        // hypothetically it should just always be -90 -> see discussion with patrick

        poseToReturn.setWristAngle(-Math.PI / 2.);

      } else {
        lastShoulderSign = Math.copySign(1, currentPose.getShoulderAngle());
      }
    }

    if (isFlipped) {

      if (Math.abs(desiredPose.getShoulderAngle()) < Math.abs(currentPose.getShoulderAngle())
          || (currentPose.getElevatorHeight() > 0.5
              || Math.abs(currentPose.getShoulderAngle())
                  > Units.degreesToRadians(15))) { // when you are GOING UP or
        // you
        // are GREATER THAN 20
        if (!isAlgae) {
          poseToReturn.setWristAngle(-desiredPose.getWristAngle());
        }
        // ABOVE SHOULD BE POSETORETURN -> CHECK
      }
    }

    // lets arm go up before elevator goes DOWN
    if (currentPose.getElevatorHeight() < desiredPose.getElevatorHeight()
        && desiredState != robotState.GROUND_INTAKE
        && Math.abs(currentPose.getShoulderAngle()) > Units.degreesToRadians(70)
        && desiredPose.getElevatorHeight() < 0.3
        && !isAlgae
        && desiredState != robotState.CLIMB
        && desiredState != robotState.CLIMB_POST
        && desiredState != robotState.VERTICAL_INTAKE) {

      poseToReturn.setElevatorHeight(currentPose.getElevatorHeight());
    } else {
    }

    // if we go to a score state

    if (desiredState == robotState.L2_SCORE
        || desiredState == robotState.L3_SCORE
        || desiredState == robotState.L4_SCORE
        || desiredState == robotState.L4_FINALE) {
      if (currentPose.getElevatorHeight() > desiredPose.getElevatorHeight()
          && Math.abs(
                  Math.abs(currentPose.getShoulderAngle())
                      - Math.abs(desiredPose.getShoulderAngle()))
              > Units.degreesToRadians(2.5)) {

        poseToReturn.setElevatorHeight(currentPose.getElevatorHeight());
      }
    }

    // if intake isn't in place yet, wait for arm (if you're heading down)
    if (Math.abs(desiredPose.getCoralIntakeAngle() - currentPose.getCoralIntakeAngle())
            > Units.degreesToRadians(70.)
        && Math.abs(desiredPose.getShoulderAngle()) > Math.abs(currentPose.getShoulderAngle())
        && desiredPose.getShoulderAngle() > 0) {

      poseToReturn.setShoulderAngle(currentPose.getShoulderAngle());
    }

    // wait to run intakes until both are down
    if (Math.abs(desiredPose.getCoralIntakeAngle() - currentPose.getCoralIntakeAngle())
            > Units.degreesToRadians(6.5)
        || Math.abs(
                Math.abs(desiredPose.getShoulderAngle()) - Math.abs(currentPose.getShoulderAngle()))
            > Units.degreesToRadians(3.)) {
      poseToReturn.setGripperSpeed(0.);
      poseToReturn.setCoralIntakeSpeed(0);
    }
    if (Math.abs(currentPose.getCoralIntakeAngle()) < Units.degreesToRadians(20)
        && desiredState != robotState.GROUND_INTAKE) {
      poseToReturn.setCoralIntakeSpeed(1.);
    }

    // if elevator is going up, keep shoulder at 0
    if (desiredState != robotState.ALGAE_FLINGUP && desiredState != robotState.GROUND_INTAKE) {
      if (desiredState == robotState.GROUND_INTAKE) {
        if (Math.abs(desiredPose.getElevatorHeight() - currentPose.getElevatorHeight())
                > Units.inchesToMeters(7)
            && desiredPose.getElevatorHeight() > currentPose.getElevatorHeight()
            && currentPose.getElevatorHeight() < 0.55) {
          poseToReturn.setShoulderAngle(0);
        }
      } else {
        if (Math.abs(desiredPose.getElevatorHeight() - currentPose.getElevatorHeight())
                > Units.inchesToMeters(3)
            && desiredPose.getElevatorHeight() > currentPose.getElevatorHeight()
            && currentPose.getElevatorHeight() < 0.55) {
          poseToReturn.setShoulderAngle(0);
        }
      }
    }

    // if you're going down from a score position, set shoulder to 0
    if (desiredState != robotState.L1_SCORE
        && desiredState != robotState.L1_INVERTED
        && desiredState != robotState.L2_SCORE
        && desiredState != robotState.L3_SCORE
        && desiredState != robotState.L4_SCORE
        && desiredState != robotState.L4_FINALE
        && desiredState != robotState.ALGAE_GROUND_INTAKE
        && desiredState != robotState.ALGAE_UPRIGHT_INTAKE
        && desiredState != robotState.PROCESSOR
        && desiredState != robotState.GROUND_INTAKE) {

      if (!isAlgae
          && Math.abs(desiredPose.getElevatorHeight() - currentPose.getElevatorHeight())
              > Units.inchesToMeters(2)
          && desiredPose.getElevatorHeight() < currentPose.getElevatorHeight()) {
        poseToReturn.setShoulderAngle(0.);
      }
    }

    if (isAlgae
        && desiredPose.getElevatorHeight() < currentPose.getElevatorHeight()
        && Math.abs(currentPose.getShoulderAngle() - desiredPose.getShoulderAngle())
            > Units.degreesToRadians(5)) {
      poseToReturn.setElevatorHeight(currentPose.getElevatorHeight());
    }
    // flip checks for stow
    if (isFlipped
        && firstFlippedPose
        && (desiredState == robotState.STOW || desiredState == robotState.INITIAL_STOW)) {
      if (currentPose.getElevatorHeight() > 0.5 && !isAlgae) {
        poseToReturn.setWristAngle(-desiredPose.getWristAngle());
      }
    }

    if (firstFlippedPose
        && (desiredState == robotState.STOW || desiredState == robotState.INITIAL_STOW)) {
      poseToReturn.setElevatorHeight(0.7);
    }
    if (desiredState.equals(robotState.VERTICAL_INTAKE)) {
      if (currentPose.getShoulderAngle() > Units.degreesToRadians(65)
          && !MathUtil.isNear(
              RobotPoseDefinitions.VERTICAL_INTAKE.getWristAngle(),
              currentPose.getWristAngle(),
              Units.degreesToRadians(3))) {
        poseToReturn.setShoulderAngle(currentPose.getShoulderAngle());
      }
    }
    // the stow check
    if (!isAlgae) {
      if (!(currentPose.getElevatorHeight() > 0.5
          || Math.abs(currentPose.getShoulderAngle()) > Units.degreesToRadians(15))) {
        poseToReturn.setWristAngle(Math.copySign(Math.PI / 2., currentPose.getWristAngle()));
      }
    }

    if (!isAlgae) {
      if (desiredState == robotState.L1_PRESCORE || desiredState == robotState.L1_SCORE) {
        if (Math.abs(currentPose.getElevatorHeight() - desiredPose.getElevatorHeight()) < 0.1
            && Math.abs(currentPose.getWristAngle() - desiredPose.getWristAngle())
                > Units.degreesToRadians(1)
            && Math.abs(currentPose.getShoulderAngle()) > Units.degreesToRadians(18)
            && currentPose.getElevatorHeight() > .55) {
          poseToReturn.setShoulderAngle(currentPose.getShoulderAngle());
        }

        if (Math.abs(currentPose.getElevatorHeight() - desiredPose.getElevatorHeight()) > 0.1) {
          poseToReturn.setWristAngle(-Math.PI / 2.);
        }
      }
    }

    if (!isAlgae
        && currentState != robotState.GROUND_INTAKE
        && currentState != robotState.PLAYER_INTAKE) {
      if ((desiredState == robotState.L2_PRESCORE
              || desiredState == robotState.L2_SCORE
              || desiredState == robotState.L2_PRESCORE_ZERO)
          && Math.abs(
                  currentPose.getWristAngle()
                      - (isFlipped ? -desiredPose.getWristAngle() : desiredPose.getWristAngle()))
              > Units.degreesToRadians(3)
          && Math.abs(currentPose.getShoulderAngle()) > Units.degreesToRadians(18)
          && Math.abs(currentPose.getShoulderAngle()) < Units.degreesToRadians(24)) {

        Logger.recordOutput("Superstructure/this wrist check", true);
        poseToReturn.setShoulderAngle(currentPose.getShoulderAngle());
      } else {
        Logger.recordOutput("Superstructure/this wrist check", false);
        Logger.recordOutput(
            "Superstructure/;(",
            Math.abs(currentPose.getWristAngle() - desiredPose.getWristAngle())
                    > Units.degreesToRadians(1)
                && Math.abs(currentPose.getShoulderAngle()) > Units.degreesToRadians(20)
                && Math.abs(currentPose.getShoulderAngle()) < Units.degreesToRadians(90));
      }
    }

    // end of kevin checks
    // --------------------------------------------------------------------

    Logger.recordOutput(
        "SuperStructure/desired elevator postchecks", poseToReturn.getElevatorHeight());
    Logger.recordOutput(
        "SuperStructure/desired shoulder postchecks", poseToReturn.getShoulderAngle());
    Logger.recordOutput("SuperStructure/desired wrist postchecks", poseToReturn.getWristAngle());
    Logger.recordOutput(
        "SuperStructure/desired coral postchecks", poseToReturn.getCoralIntakeAngle());
    Logger.recordOutput("SuperStructure/desired coral speed", poseToReturn.getCoralIntakeSpeed());
    Logger.recordOutput("SuperStructure/shoulder sign", lastShoulderSign);

    currentDesiredPose =
        new RobotPose(
            poseToReturn.getElevatorHeight(),
            poseToReturn.getShoulderAngle(),
            poseToReturn.getWristAngle(),
            poseToReturn.getCoralIntakeAngle(),
            poseToReturn.getCoralIntakeSpeed(),
            desiredPose.getGripperSpeed());

    return new RobotPose(
            poseToReturn.getElevatorHeight(),
            poseToReturn.getShoulderAngle(),
            poseToReturn.getWristAngle(),
            poseToReturn.getCoralIntakeAngle(),
            poseToReturn.getCoralIntakeSpeed(),
            desiredPose.getGripperSpeed())
        .setShoulderConfigs(desiredPose.getShoulderConfigs())
        .setEleavtorConfigs(desiredPose.getElevatorConfigs());
  }

  public RobotPose getOriginalDesiredPose(robotState state) {
    switch (state) {
      case GROUND_INTAKE:
        return RobotPoseDefinitions.GROUND_INTAKE;
      case PLAYER_INTAKE:
        return RobotPoseDefinitions.PLAYER_INTAKE;
      case L2_ALGAE_GRAB:
        return RobotPoseDefinitions.L2_ALGAE;
      case L3_ALGAE_GRAB:
        return RobotPoseDefinitions.L3_ALGAE;
      case L4_PRESCORE:
        return RobotPoseDefinitions.L4_PRESCORE;
      case L4_SCORE:
        return RobotPoseDefinitions.L4_SCORE;
      case L3_PRESCORE:
        return RobotPoseDefinitions.L3_PRESCORE;
      case L3_SCORE:
        return RobotPoseDefinitions.L3_SCORE;
      case L2_PRESCORE:
        return RobotPoseDefinitions.L2_PRESCORE;
      case L2_SCORE:
        return RobotPoseDefinitions.L2_SCORE;
      case L1_PRESCORE:
        return RobotPoseDefinitions.L1_PRESCORE;
      case L1_SCORE:
        return RobotPoseDefinitions.L1_SCORE;
      case L1_INVERTED:
        return RobotPoseDefinitions.L1_INVERTED;
      case L4_PRESCORE_ZERO:
        return RobotPoseDefinitions.L4_PRESCORE_ZERO;
      case L3_PRESCORE_ZERO:
        return RobotPoseDefinitions.L3_PRESCORE_ZERO;
      case L2_PRESCORE_ZERO:
        return RobotPoseDefinitions.L2_PRESCORE_ZERO;
      case L4_FINALE:
        return RobotPoseDefinitions.L4_AUTO_FINALE;
      case STOW:
        return RobotPoseDefinitions.STOW;
      case INITIAL_STOW:
        return RobotPoseDefinitions.INITIAL_STOW;
      case CLIMB_POST:
        return RobotPoseDefinitions.CLIMB_POST;
      case CLIMB:
        return RobotPoseDefinitions.CLIMB;
      case PROCESSOR:
        return RobotPoseDefinitions.PROCESSOR;
      case L1_DOWNWARD:
        return RobotPoseDefinitions.L1_DOWNWARD;
      case ALGAE_HOLDING:
        return RobotPoseDefinitions.ALGAE_HOLDING;
      case ALGAE_GROUND_INTAKE:
        return RobotPoseDefinitions.ALGAE_GROUND_INTAKE;
      case STOW_FOR_ALGAE_AUTO:
        return RobotPoseDefinitions.STOW_FOR_ALGAE_AUTO;
      case ALGAE_UPRIGHT_INTAKE:
        return RobotPoseDefinitions.ALGAE_UPRIGHT_INTAKE;
      case ALGAE_FLING:
        return RobotPoseDefinitions.ALGAE_FLING;
      case VERTICAL_INTAKE:
        return RobotPoseDefinitions.VERTICAL_INTAKE;
      default:
        return new RobotPose(0, 0, 0, 0, 0, 0);
    }
  }

  public void setDesiredState(robotState state) {
    desiredState = state;
    Logger.recordOutput("SuperStructure/Desired State", state);
  }

  public boolean isAtDesiredPose(RobotPose originalDesiredPose) {
    double desiredShoulder = originalDesiredPose.getShoulderAngle();
    double desiredElevator = originalDesiredPose.getElevatorHeight();
    if (!isFront) {
      if (desiredState == robotState.CLIMB || desiredState == robotState.CLIMB_POST) {
        desiredShoulder = desiredShoulder;
      }
      desiredShoulder = -desiredShoulder;
    }
    if (firstFlippedPose) {
      if (desiredState == robotState.L3_PRESCORE || desiredState == robotState.L4_PRESCORE) {
        desiredShoulder = 0;
      }
      desiredShoulder = -desiredShoulder;
    }

    if (firstFlippedPose
        && (desiredState == robotState.STOW || desiredState == robotState.INITIAL_STOW)) {
      desiredElevator = (0.7);
    }

    double currentElevator = currentPose.getElevatorHeight();
    double currentShoulder = currentPose.getShoulderAngle();
    double currentWrist = currentPose.getWristAngle();
    double currentCoral = currentPose.getCoralIntakeAngle();

    boolean elevatorInTol = MathUtil.isNear(desiredElevator, currentElevator, 0.1);
    boolean shoulderInTol = MathUtil.isNear(desiredShoulder, currentShoulder, 0.1);
    boolean wristInTol =
        MathUtil.isNear(
            Math.copySign(originalDesiredPose.getWristAngle(), currentWrist),
            currentWrist,
            Units.degreesToRadians(2));
    boolean coralInTol =
        MathUtil.isNear(originalDesiredPose.getCoralIntakeAngle(), currentCoral, 0.1);

    return elevatorInTol && shoulderInTol && wristInTol && coralInTol;
  }

  public Command createStateCommand(robotState originalDesiredState) {
    return Commands.run(() -> setDesiredState(originalDesiredState))
        .until(() -> isAtDesiredPose(getOriginalDesiredPose(originalDesiredState)));
  }

  public Command createStateCommand2(Supplier<robotState> originalDesiredState) {
    return Commands.run(() -> setDesiredState(originalDesiredState.get()))
        .until(() -> isAtDesiredPose(getOriginalDesiredPose(originalDesiredState.get())));
  }

  public RobotPose getDesiredPose(RobotPose currentPose) {

    currentState = desiredState;

    if (currentState == robotState.GROUND_INTAKE) {
      if (isAlgae) {
        currentState = robotState.ALGAE_GROUND_INTAKE;
        desiredState = currentState;
        return poseAdjust(RobotPoseDefinitions.ALGAE_GROUND_INTAKE, currentPose);
      }
      return poseAdjust(RobotPoseDefinitions.GROUND_INTAKE, currentPose);

    } else if (currentState == robotState.PLAYER_INTAKE) {
      if (isAlgae) {
        currentState = robotState.ALGAE_UPRIGHT_INTAKE;
        desiredState = currentState;
        return poseAdjust(RobotPoseDefinitions.ALGAE_UPRIGHT_INTAKE, currentPose);
      }
      return poseAdjust(RobotPoseDefinitions.PLAYER_INTAKE, currentPose);

    } else if (currentState == robotState.L4_PRESCORE) {
      Logger.recordOutput(
          "SuperStructure/L4_PRESCORE VALUE", RobotPoseDefinitions.L4_PRESCORE.getWristAngle());

      return poseAdjust(RobotPoseDefinitions.L4_PRESCORE, currentPose);

    } else if (currentState == robotState.L3_PRESCORE) {
      if (isAlgae) {
        currentState = robotState.L3_ALGAE_GRAB;
        desiredState = currentState;
        return poseAdjust(RobotPoseDefinitions.L3_ALGAE, currentPose);
      }
      return poseAdjust(RobotPoseDefinitions.L3_PRESCORE, currentPose);

    } else if (currentState == robotState.L2_PRESCORE) {

      if (isAlgae) {
        currentState = robotState.L2_ALGAE_GRAB;
        desiredState = currentState;
        return poseAdjust(RobotPoseDefinitions.L2_ALGAE, currentPose);
      }
      return poseAdjust(RobotPoseDefinitions.L2_PRESCORE, currentPose);

    } else if (currentState == robotState.L1_PRESCORE) {
      if (!isFront && !isAlgae) {
        currentState = robotState.L1_PRESCORE;
        desiredState = currentState;
        return poseAdjust(RobotPoseDefinitions.L1_PRESCORE, currentPose);
      } else if (isAlgae) {
        currentState = robotState.PROCESSOR;
        desiredState = currentState;
        return poseAdjust(RobotPoseDefinitions.PROCESSOR, currentPose);
      } else {
        currentState = robotState.L1_PRESCORE;
        desiredState = currentState;
        return poseAdjust(RobotPoseDefinitions.L1_PRESCORE, currentPose);
      }

    } else if (currentState == robotState.L4_SCORE) {
      return poseAdjust(RobotPoseDefinitions.L4_SCORE, currentPose);
    } else if (currentState == robotState.L4_FINALE) {
      return poseAdjust(RobotPoseDefinitions.L4_AUTO_FINALE, currentPose);
    } else if (currentState == robotState.L3_SCORE) {
      return poseAdjust(RobotPoseDefinitions.L3_SCORE, currentPose);
    } else if (currentState == robotState.L2_SCORE) {
      return poseAdjust(RobotPoseDefinitions.L2_SCORE, currentPose);

    } else if (currentState == robotState.L1_SCORE) {
      return poseAdjust(RobotPoseDefinitions.L1_SCORE, currentPose);

    } else if (currentState == robotState.L2_ALGAE_GRAB) {
      if (isAlgae) {
        currentState = robotState.L2_ALGAE_GRAB;
        desiredState = currentState;
        return poseAdjust(RobotPoseDefinitions.L2_ALGAE, currentPose);
      } else {
        currentState = robotState.L2_PRESCORE;
        desiredState = currentState;
        return poseAdjust(RobotPoseDefinitions.L2_PRESCORE, currentPose);
      }

    } else if (currentState == robotState.L3_ALGAE_GRAB) {
      if (isAlgae) {
        currentState = robotState.L3_ALGAE_GRAB;
        desiredState = currentState;
        return poseAdjust(RobotPoseDefinitions.L3_ALGAE, currentPose);
      } else {
        currentState = robotState.L3_PRESCORE;
        desiredState = currentState;
        return poseAdjust(RobotPoseDefinitions.L3_PRESCORE, currentPose);
      }
    } else if (currentState == robotState.ALGAE_UPRIGHT_INTAKE) {
      if (isAlgae) {
        currentState = robotState.ALGAE_UPRIGHT_INTAKE;
        desiredState = currentState;
        return poseAdjust(RobotPoseDefinitions.ALGAE_UPRIGHT_INTAKE, currentPose);
      } else {
        currentState = robotState.PLAYER_INTAKE;
        desiredState = currentState;
        return poseAdjust(RobotPoseDefinitions.L3_PRESCORE, currentPose);
      }
    } else if (currentState == robotState.STOW) {
      if (isAlgae) {
        currentState = robotState.ALGAE_HOLDING;
        desiredState = currentState;
        return poseAdjust(RobotPoseDefinitions.ALGAE_HOLDING, currentPose);
      }
      return poseAdjust(RobotPoseDefinitions.STOW, currentPose);

    } else if (currentState == robotState.INITIAL_STOW) {
      if (isAlgae) {
        currentState = robotState.ALGAE_HOLDING;
        desiredState = currentState;
        return poseAdjust(RobotPoseDefinitions.ALGAE_HOLDING, currentPose);
      }
      return poseAdjust(RobotPoseDefinitions.INITIAL_STOW, currentPose);

    } else if (currentState == robotState.CLIMB) {
      return poseAdjust(RobotPoseDefinitions.CLIMB, currentPose);
    } else if (currentState == robotState.CLIMB_POST) {
      return poseAdjust(RobotPoseDefinitions.CLIMB_POST, currentPose);

    } else if (currentState == robotState.STOW_FOR_ALGAE_AUTO) {
      return poseAdjust(RobotPoseDefinitions.STOW_FOR_ALGAE_AUTO, currentPose);
    } else if (currentState == robotState.PROCESSOR) {
      return poseAdjust(RobotPoseDefinitions.PROCESSOR, currentPose);
    } else if (currentState == robotState.L1_DOWNWARD) {
      return poseAdjust(RobotPoseDefinitions.L1_DOWNWARD, currentPose);
    } else if (currentState == robotState.ALGAE_HOLDING) {
      if (isAlgae) {
        currentState = robotState.ALGAE_HOLDING;
        desiredState = currentState;
        return poseAdjust(RobotPoseDefinitions.ALGAE_HOLDING, currentPose);
      } else {
        currentState = robotState.STOW;
        desiredState = currentState;
        return poseAdjust(RobotPoseDefinitions.STOW, currentPose);
      }

    } else if (currentState == robotState.BARGE) {
      return poseAdjust(RobotPoseDefinitions.BARGE, currentPose);
    } else if (currentState == robotState.L4_PRESCORE_ZERO) {
      return poseAdjust(RobotPoseDefinitions.L4_PRESCORE_ZERO, currentPose);
    } else if (currentState == robotState.L3_PRESCORE_ZERO) {
      if (isAlgae) {
        currentState = robotState.L3_ALGAE_GRAB;
        desiredState = currentState;
        return poseAdjust(RobotPoseDefinitions.L3_ALGAE, currentPose);
      }

      return poseAdjust(RobotPoseDefinitions.L3_PRESCORE_ZERO, currentPose);
    } else if (currentState == robotState.L2_PRESCORE_ZERO) {

      if (isAlgae) {
        currentState = robotState.L2_ALGAE_GRAB;
        desiredState = currentState;
        return poseAdjust(RobotPoseDefinitions.L2_ALGAE, currentPose);
      }
      return poseAdjust(RobotPoseDefinitions.L2_PRESCORE_ZERO, currentPose);

    } else if (currentState == robotState.ALGAE_FLING) {
      return poseAdjust(RobotPoseDefinitions.ALGAE_FLING, currentPose);
    } else if (currentState == robotState.ALGAE_FLINGUP) {
      return poseAdjust(RobotPoseDefinitions.ALGAE_FLINGUP, currentPose);
    } else if (currentState == robotState.ALGAE_GROUND_INTAKE) {
      if (isAlgae) {
        currentState = robotState.ALGAE_GROUND_INTAKE;
        desiredState = currentState;
        return poseAdjust(RobotPoseDefinitions.ALGAE_GROUND_INTAKE, currentPose);
      } else {
        currentState = robotState.GROUND_INTAKE;
        desiredState = currentState;
        return poseAdjust(RobotPoseDefinitions.GROUND_INTAKE, currentPose);
      }

    } else if (currentState == robotState.VERTICAL_INTAKE) {
      return poseAdjust(RobotPoseDefinitions.VERTICAL_INTAKE, currentPose);
    } else {
      return poseAdjust(new RobotPose(0, 0, 0), currentPose);
    }
  }
}
