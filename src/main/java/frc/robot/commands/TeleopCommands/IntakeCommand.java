// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotVisualizer;
import frc.robot.subsystems.SuperStructure.Superstructure;
import frc.robot.subsystems.SuperStructure.Superstructure.robotState;
import frc.robot.subsystems.gripper.Gripper;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCommand extends Command {
  /** Creates a new ScoreUntilOuttake. */
  private Gripper gripper;

  private Superstructure struct;

  private boolean isAlgae;

  private robotState state;

  public IntakeCommand(Superstructure struct, Gripper gripper, robotState state) {
    this.struct = struct;
    this.state = state;
    this.gripper = gripper;
    addRequirements(gripper);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    isAlgae = (state != robotState.GROUND_INTAKE && state != robotState.PLAYER_INTAKE);
    struct.setDesiredState(state);
    gripper.runVolts(() -> -12);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (gripper.hasPiece().getAsBoolean()) {
      RobotVisualizer.setHasAlgae(isAlgae);
      struct.setDesiredState(isAlgae ? robotState.ALGAE_HOLDING : robotState.STOW);
    } else {
      struct.setDesiredState(isAlgae ? robotState.ALGAE_HOLDING : robotState.STOW);
    }

    if (isAlgae) {
      gripper.runVolts(() -> -1);
    } else {
      gripper.runVolts(() -> 0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return gripper.hasPiece().getAsBoolean() && !isAlgae;
  }
}
