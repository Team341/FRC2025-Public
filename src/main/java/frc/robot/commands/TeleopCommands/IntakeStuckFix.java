// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.motor_factories.elevator.Elevator;
import frc.robot.subsystems.gripper.Gripper;

/** designed to be while held */
public class IntakeStuckFix extends Command {
  /** Creates a new IntakeStuckFix. */
  Gripper gripper;

  Elevator elevator;

  public IntakeStuckFix(Elevator elevator, Gripper gripper) {

    addRequirements(elevator, gripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.runVolts(2);
    gripper.runVolts(() -> 12);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    elevator.runVolts(0);
    gripper.runVolts(() -> 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
