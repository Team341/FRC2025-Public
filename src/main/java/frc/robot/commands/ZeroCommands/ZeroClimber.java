// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ZeroCommands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.motor_factories.elevator.Elevator;
import frc.robot.subsystems.climber.Climber;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead
https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ZeroClimber extends Command {
  /** Creates a new ResetZero. */
  private Climber climber;

  private Elevator elevator;

  private double velocityThreshold = 5;
  private boolean velocityZero = false;
  private Debouncer veloDebouncer;

  public ZeroClimber(Climber mClimber, Elevator mElevator) {
    climber = mClimber;
    elevator = mElevator;
    // Use addRequirements() here to declare subsystem dependencies.
    veloDebouncer = new Debouncer(.5);
    addRequirements(climber, mElevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.runPosition(() -> Units.inchesToMeters(4));
    velocityZero = veloDebouncer.calculate(Math.abs(climber.getVelocity()) < velocityThreshold);
    Logger.recordOutput("Climber/Debounced Velocity Zero", velocityZero);
    if (elevator.atPosition(0.01)) {
      climber.runVolts(() -> -2.);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.runVolts(() -> 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (climber.getLimit()) {
      climber.resetAngle(Units.degreesToRadians(0));
      return true;
    }
    return false;
  }
}
