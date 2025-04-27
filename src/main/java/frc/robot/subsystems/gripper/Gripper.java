// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.SuperStructure.Superstructure;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Gripper extends SubsystemBase {

  private GripperIOInputsAutoLogged inputs = new GripperIOInputsAutoLogged();
  private GripperIO io;
  private static Gripper instance;
  private Debouncer debouncer;
  private Debouncer debouncerLimit;
  private DoubleSupplier shoulderAngle;
  private boolean debouncedVelocityVoltage;
  private boolean debouncedLimitSwitch;

  public static Gripper getInstance(DoubleSupplier shoulderAngle) {
    if (instance == null) {
      return instance = new Gripper(shoulderAngle);
    } else {
      return instance;
    }
  }

  /** Creates a new Gripper. */
  public Gripper(DoubleSupplier shoulderAngle) {
    this.shoulderAngle = shoulderAngle;
    switch (Constants.currentMode) {
      case REAL:
        io = new GripperIOTalonFX(shoulderAngle);
        break;
      case SIM:
        io = new GripperIOSim(shoulderAngle);
        break;
      case REPLAY:
        io = new GripperIO() {};
        break;
      default:
        System.out.println("Gripper not created correctly");
        break;
    }
    debouncer = new Debouncer(0.075); // Debounce time can be tuned
    debouncerLimit = new Debouncer(0.1); // Debounce time can be tuned
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Gripper", inputs);

    debouncedLimitSwitch = debouncerLimit.calculate(inputs.beamBreak);

    debouncedVelocityVoltage =
        debouncer.calculate(
            Math.abs(inputs.appliedVolts) >= 1 && MathUtil.isNear(-3750, inputs.velocityRPM, 400));
    if (getTemp() > 90) {
      halt();
    }
  }

  public boolean getBeamBreak() {
    return inputs.beamBreak;
  }

  public void setFling(Boolean isFlingMode) {
    io.setFling(isFlingMode);
  }

  public void runVolts(DoubleSupplier volts) {
    io.runVolts(volts.getAsDouble());
  }

  public void halt() {
    io.stop();
  }

  public double getTemp() {
    return inputs.temperatureCelsius;
  }

  /**
   * This method returns a trigger that is true when the limit switch is true, or if thhe voltage of
   * the gripper is above kS but the velocity is close to zero
   */
  public Trigger hasPiece() {
    return new Trigger(
        () -> {
          if (Superstructure.getInstance().isAlgae) {
            return debouncedLimitSwitch || debouncedVelocityVoltage;
          } else return debouncedLimitSwitch;
        });
  }

  public void setMode(NeutralModeValue mode) {
    io.setMode(mode);
  }

  public boolean notifierCalled() {
    return inputs.notifierCalled;
  }
}
