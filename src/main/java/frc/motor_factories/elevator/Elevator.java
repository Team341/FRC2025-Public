// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.motor_factories.elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private String name = "Elevator";
  private ElevatorIO io;
  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  public static boolean hasBeenZeroed = false;

  private static Elevator instance;

  public static Elevator getInstance(ElevatorDescription description) {
    if (instance == null) {
      return new Elevator(description);
    } else {
      return instance;
    }
  }

  /** Creates a new Elevator. */
  public Elevator(ElevatorDescription elevator) {
    this(elevator, "Elevator");
  }

  public Elevator(ElevatorDescription elevator, String name) {
    this.name = name;
    switch (Constants.currentMode) {
      case REAL:
        this.io = new ElevatorIOTalonFX(elevator, name);
        break;
      case SIM:
        this.io = new ElevatorIOSim(name, elevator);
        break;
      case REPLAY:
        this.io = new ElevatorIO() {};
        break;
      default:
        System.out.println("Bad robot mode in elevator");
        break;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs(this.name, inputs);
    Logger.recordOutput("Elevator", atPosition(.1));

    if (!hasBeenZeroed && limitSwitchPressed()) {
      hasBeenZeroed = true;
      resetPosition(0.);
      runVolts(0.);
    }
    Logger.recordOutput("Elevator/hasbeenzeroed", hasBeenZeroed);
  }

  public void resetPosition(double position) {
    io.resetPosition(position);
  }

  public void stop() {
    io.stop();
  }

  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  public boolean limitSwitchPressed() {
    return inputs.lowerLimitSwitchHit;
  }

  public double getVelocity() {
    return inputs.masterVelocity;
  }

  public double getPosition() {
    return inputs.positionMeters;
  }

  public double getVolts() {
    return inputs.masterAppliedVolts;
  }

  public void runPosition(DoubleSupplier distance) {
    inputs.desiredPosition = distance.getAsDouble();
    io.setPosition(distance.getAsDouble());
  }

  public boolean atPosition(double tolerance) {
    return MathUtil.isNear(inputs.desiredPosition, inputs.positionMeters, tolerance);
  }

  public void setPID(double kP, double kI, double kD) {}

  public Command runHeight(DoubleSupplier height) {
    return Commands.run(() -> runPosition(() -> height.getAsDouble()), this);
  }

  public void setHeightDynamic(DoubleSupplier height, Supplier<MotionMagicConfigs> config) {
    io.setPositionDynamic(
        height.getAsDouble(),
        config.get().MotionMagicCruiseVelocity,
        config.get().MotionMagicAcceleration,
        config.get().MotionMagicJerk);
    ;
  }

  public Command halt() {
    return Commands.run(() -> stop(), this);
  }
}
