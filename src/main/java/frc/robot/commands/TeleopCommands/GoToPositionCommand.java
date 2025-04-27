// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SuperStructure.Superstructure;
import frc.robot.subsystems.SuperStructure.Superstructure.robotState;
import frc.robot.subsystems.gripper.Gripper;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToPositionCommand extends Command {
  /** Creates a new ScoreUntilOuttake. */
  private Gripper gripper;

  private Superstructure struct;

  private boolean isAlgae;

  private robotState state;

  private boolean requiresZero = false;

  private boolean useZero;

  public GoToPositionCommand(
      Superstructure struct, Gripper gripper, robotState state, boolean useZero) {
    this.struct = struct;
    this.state = state;
    this.gripper = gripper;
    this.useZero = useZero;
    addRequirements(gripper);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.

  int debounceCount = 0;

  @Override
  public void initialize() {

    switch (state) {
      case L4_PRESCORE:
        this.requiresZero = state == robotState.L4_PRESCORE;
        break;
      case L3_PRESCORE:
        this.requiresZero = state == robotState.L3_PRESCORE;
        break;
      case L2_PRESCORE:
        this.requiresZero = state == robotState.L2_PRESCORE;
        break;
      default:
        this.requiresZero = false;
        break;
    }
    if (!useZero) {
      this.requiresZero = false;
    }
    // Logger.recordOutput("Superstructure/RequiresZero", requiresZero);
    isAlgae = struct.isAlgae;
    debounceCount = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    isAlgae = struct.isAlgae;

    if (!isAlgae) {
      if (this.requiresZero == true) {
        switch (state) {
          case L4_PRESCORE:
            state = robotState.L4_PRESCORE_ZERO;
            break;
          case L3_PRESCORE:
            state = robotState.L3_PRESCORE_ZERO;
            break;
          case L2_PRESCORE:
            state = robotState.L2_PRESCORE_ZERO;
            break;
          default:
            state = state;
            break;
        }
      }
    }

    struct.setDesiredState(state);

    if (isAlgae && state != robotState.STOW) {
      gripper.runVolts(() -> -12.);
    }

    if (isAlgae && gripper.hasPiece().getAsBoolean()) {
      debounceCount++;
    } else {
      debounceCount = 0;
    }
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (state == robotState.STOW && gripper.hasPiece().getAsBoolean() && isAlgae) {
      gripper.runVolts(() -> -4.0);
    } else if (isAlgae && state != robotState.STOW) {
      if (state == robotState.L3_PRESCORE) {
        struct.setDesiredState(robotState.L3_PRESCORE);
      } else if (state == robotState.L2_PRESCORE) {
        struct.setDesiredState(robotState.L2_PRESCORE);
      } else {
        struct.setDesiredState(robotState.STOW);
      }

      gripper.runVolts(() -> -4.0);

    } else {
      gripper.runVolts(() -> 0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (isAlgae) {
      return debounceCount > 40;

    } else {
      return struct.isAtDesiredPose(struct.getOriginalDesiredPose(state));
    }
  }
}
