// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ZeroCommands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.motor_factories.elevator.Elevator;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.subsystems.wrist.Wrist;

/* You should consider using the more terse Command factories API instead
https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ZeroWrist extends Command {
  /** Creates a new ResetZero. */
  private Wrist wrist;

  private Shoulder shoulder;
  private Elevator elevator;

  private double velocityThreshold = 5;
  private boolean velocityZero = false;
  private Debouncer veloDebouncer;

  public ZeroWrist(Wrist mWrist, Shoulder mShoulder, Elevator mElevator) {
    wrist = mWrist;
    shoulder = mShoulder;
    elevator = mElevator;
    // Use addRequirements() here to declare subsystem dependencies.
    veloDebouncer = new Debouncer(.1);
    addRequirements(wrist, shoulder, mElevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    velocityZero = veloDebouncer.calculate(Math.abs(wrist.getVelocityRPM()) < velocityThreshold);

    shoulder.setAngle(() -> 0);
    elevator.runHeight(() -> Units.inchesToMeters(4));

    wrist.runVolts(() -> -.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.runVolts(() -> 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (velocityZero && Math.abs(wrist.getAppliedVolts()) > .25) {
      wrist.resetAngle(Units.degreesToRadians(-102)); // 104.5?
      return true;
    }
    return false;
  }
}
