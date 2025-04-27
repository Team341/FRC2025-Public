// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private ClimberIO io;

  private static Climber instance;

  public static Climber getInstance() {
    if (instance == null) return instance = new Climber();
    return instance;
  }

  public Climber() {
    switch (Constants.currentMode) {
      case REAL:
        io = new ClimberIOTalonFX();
        io.setMM(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(75)
                .withMotionMagicAcceleration(250));
        io.setPID(new Slot0Configs().withKS(0.625).withKP(1));
        System.out.println("real created");
        io.resetAngle(0);

        break;
      case SIM:
        io = new ClimberIOSim();
        io.setPID(new Slot0Configs().withKP(0.5));
        System.out.println("Sim created");
        break;
      case REPLAY:
        io = new ClimberIO() {};
        break;
      default:
        System.out.println("Climber not created properly");
        break;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }

  public void halt() {
    io.stop();
  }

  public void runAngle(double angle) {
    io.runAngle(angle);
  }

  public void runVolts(DoubleSupplier volts) {
    io.runVolts(volts.getAsDouble());
  }

  public void runVoltsAndSetAngle(double angle, double volts) {
    io.runAngle(angle);
    io.setGrabberVolts(volts);
  }

  public Command setAngle(DoubleSupplier angle) {
    return Commands.run(() -> io.runAngle(angle.getAsDouble()), instance);
  }

  public Command setVolts(double volts) {
    return Commands.run(() -> io.runVolts(volts), instance);
  }

  public double getAngle() {
    return Units.degreesToRadians(inputs.positionDeg);
  }

  public double getVelocity() {
    return inputs.velocityRPM;
  }

  public void resetAngle(double angle) {
    io.resetAngle(angle);
  }

  public void setGrabberVolts(double volts) {
    io.setGrabberVolts(volts);
  }

  public boolean getLimit() {
    return inputs.limitSwitch;
  }

  public double getVoltage() {
    return inputs.appliedVolts;
  }
}
