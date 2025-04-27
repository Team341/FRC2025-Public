// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ZeroCommands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.subsystems.wrist.Wrist;

/* You should consider using the more terse Command factories API instead
https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ZeroWristBB extends Command {
  /** Creates a new ResetZero. */
  private Wrist wrist;

  private Shoulder shoulder;

  private double velocityThreshold = 5;
  private boolean velocityZero = false;
  private Debouncer veloDebouncer;

  private boolean wristZeroed;
  private boolean intakeNotHit = false;

  public ZeroWristBB(Wrist mWrist, Shoulder mShoulder) {
    wrist = mWrist;
    shoulder = mShoulder;
    // Use addRequirements() here to declare subsystem dependencies.
    veloDebouncer = new Debouncer(.1);
    addRequirements(wrist, shoulder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("wrist zero sched");
    velocityZero = false;
    intakeNotHit = false;
    wristZeroed = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (intakeNotHit = true) {
      velocityZero = veloDebouncer.calculate(Math.abs(wrist.getVelocityRPM()) < velocityThreshold);
    }
    // Logger.recordOutput("Wrist/Debounced Velocity Zero", velocityZero);
    shoulder.setAngle(() -> 0);

    if (intakeNotHit == false) {
      if (!wrist.getBeamBreak()) {
        intakeNotHit = true;
      } else {
        wrist.runVolts(() -> -.5);
      }
    }

    if (intakeNotHit = true) {
      wrist.runVolts(() -> .5);
    }

    if (intakeNotHit && wrist.getBeamBreak()) {
      wristZeroed = true;
    }

    System.out.println("wrist zero sched");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.runVolts(() -> 0);

    if (wristZeroed) {
      wrist.resetAngle(Math.PI / 2.);
    } else if (velocityZero) {
      wrist.resetAngle(Units.degreesToRadians(102));
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return wristZeroed || (velocityZero && Math.abs(wrist.getAppliedVolts()) > .25);
  }
}
