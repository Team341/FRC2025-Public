// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.motor_factories.elevator.Elevator;
import frc.robot.subsystems.coral.Coral;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.subsystems.wrist.Wrist;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FixStuck extends Command {
  /** Creates a new FixStuck. */
  Elevator elevator;

  Coral coral;
  Shoulder shoulder;
  Wrist wrist;
  Gripper gripper;

  public FixStuck(Elevator elevator, Coral coral, Shoulder shoulder, Wrist wrist, Gripper gripper) {

    this.elevator = elevator;
    this.coral = coral;
    this.shoulder = shoulder;
    this.wrist = wrist;
    this.gripper = gripper;
    addRequirements(shoulder, coral, elevator, wrist, gripper);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    coral.setMode(NeutralModeValue.Coast);
    shoulder.setMode(NeutralModeValue.Coast);
    gripper.setMode(NeutralModeValue.Coast);
    wrist.setMode(NeutralModeValue.Coast);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (elevator.getPosition() > Units.inchesToMeters(35)) {
      elevator.runPosition(() -> elevator.getPosition());
      shoulder.runAngle(0);
    } else {
      elevator.runVolts(2);
      coral.runIntakeVolts(0);
      shoulder.runVolts(() -> 0);
      gripper.runVolts(() -> 0);
      wrist.runVolts(() -> 0);
      coral.runPivotVolts(() -> 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.runVolts(0);

    coral.setMode(NeutralModeValue.Brake);
    shoulder.setMode(NeutralModeValue.Brake);
    gripper.setMode(NeutralModeValue.Brake);
    wrist.setMode(NeutralModeValue.Brake);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(shoulder.getAngle()) < Units.degreesToRadians(5);
  }
}
