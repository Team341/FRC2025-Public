// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ZeroCommands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.Coral;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead
https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ZeroCoral extends Command {
  /** Creates a new ResetZero. */
  private Coral coral;

  private boolean coralVeloDebounced = false;
  private Debouncer veloDebouncer;
  private double velocityThreshold = 1;

  public ZeroCoral(Coral mCoral) {

    coral = mCoral;
    // Use addRequirements() here to declare subsystem dependencies.
    veloDebouncer = new Debouncer(.5);
    addRequirements(coral);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    coralVeloDebounced = veloDebouncer.calculate(Math.abs(coral.getVelocity()) < velocityThreshold);
    Logger.recordOutput("Coral/Velocity Debounced", coralVeloDebounced);
    coral.runPivotVolts(() -> 1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coral.runPivotVolts(() -> 0);
  }

  // Returns true when the command should end.s
  @Override
  public boolean isFinished() {
    if ((coralVeloDebounced && Math.abs(coral.getPivotVolts()) > .5)) {
      coral.resetPosition(Units.degreesToRadians(92));
      return true;
    }
    return false;
  }
}
