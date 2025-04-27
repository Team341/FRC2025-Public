// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SuperStructure.RobotPoseDefinitions;
import frc.robot.subsystems.SuperStructure.Superstructure;
import frc.robot.subsystems.SuperStructure.Superstructure.robotState;
import frc.robot.subsystems.gripper.*;
import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Fling extends Command {
  Gripper gripper;
  Superstructure struct;
  private boolean flinging = false;
  private DoubleSupplier shoulderVelo;

  /** Creates a new Fling. */
  public Fling(Gripper gripper, Superstructure struct) {
    this.gripper = gripper;
    this.struct = struct;

    addRequirements(gripper);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  boolean gotToDesiredStateA;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    gotToDesiredStateA = false;

    gripper.runVolts(() -> -4);
    struct.setDesiredState(robotState.ALGAE_FLING);
  }

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {

    if (struct.isAtDesiredPose(RobotPoseDefinitions.ALGAE_FLING)) {
      gotToDesiredStateA = true;
    }
    if (gotToDesiredStateA) {
      struct.setDesiredState(robotState.ALGAE_FLINGUP);
    }
    // Logger.recordOutput("SuperStructure/is at desired astate A", gotToDesiredStateA);
    if (gotToDesiredStateA
        && struct.getCurrentPose().getElevatorHeight()
            > RobotPoseDefinitions.ALGAE_FLINGUP.getElevatorHeight() - 0.4) {

      gripper.runVolts(() -> 12.);
      flinging = true;

    } else if (flinging = false) {
      gripper.runVolts(() -> -4);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    gripper.runVolts(() -> 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !gripper.hasPiece().getAsBoolean();
  }
}
