// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coral;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Coral extends SubsystemBase {
  private CoralIOInputsAutoLogged inputs = new CoralIOInputsAutoLogged();
  private CoralIO io;
  private static boolean hasBeenZeroed = false;

  private static Coral instance;

  public static Coral getInstance() {
    if (instance == null) {
      return instance = new Coral();
    } else {
      return instance;
    }
  }

  /** Creates a new Coral. */
  public Coral() {
    switch (Constants.currentMode) {
      case REAL:
        io = new CoralIOTalonFX();
        io.setPivotPID(CoralConstants.REAL_PID_CONFIGS);
        break;
      case SIM:
        io = new CoralIOSim();
        io.setPivotPID(CoralConstants.SIM_PID_CONFIGS);
        hasBeenZeroed = true;
        break;
      case REPLAY:
        io = new CoralIO() {};
        break;
      default:
        System.out.println("Coral not correctly instantiated");
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Coral", inputs);
  }

  public double getPivotAngle() {
    return Units.degreesToRadians(inputs.pivotAngleDeg);
  }

  public void runIntakeVolts(double volts) {
    io.runIntakeVolts(volts);
  }

  public double getPivotVolts() {
    return inputs.pivotAppliedVolts;
  }

  public void runPivotAngle(double angle) {
    io.setPivotAngle(angle);
  }

  public void runPivotVolts(DoubleSupplier volts) {
    io.runPivotVolts(volts.getAsDouble());
  }

  public Command setIntakeVoltage(DoubleSupplier volts) {
    return Commands.run(() -> runIntakeVolts(volts.getAsDouble()), instance);
  }

  public Command setPivotAngle(DoubleSupplier angle) {
    return Commands.run(() -> runPivotAngle(angle.getAsDouble()), instance);
  }

  public void setIntakeVoltageandSetPivotAngle(DoubleSupplier angle, DoubleSupplier volts) {
    io.setPivotAngle(angle.getAsDouble());
    io.runIntakeVolts(volts.getAsDouble());
  }

  public void resetPosition(double angle) {
    io.resetAngle(angle);
  }

  public boolean pivotLimitSwitchPressed() {
    return inputs.pivotLimitSwitchPressed;
  }

  public double getVelocity() {
    return inputs.pivotVelocityRPM;
  }

  public double getCurrent() {
    return inputs.pivotTorqueCurrent;
  }

  public boolean getLimit() {
    return inputs.pivotLimitSwitchPressed;
  }

  public void setMode(NeutralModeValue mode) {
    io.setMode(mode);
  }
}
