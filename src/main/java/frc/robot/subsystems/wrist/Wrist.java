// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {

  private WristIO io;
  private WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
  private static Wrist instance;
  Boolean hasBeenZeroed = false;

  public static Wrist getInstance() {
    if (instance == null) {
      System.out.println("instance null");
      return instance = new Wrist();
    } else {
      return instance;
    }
  }

  /** Creates a new Wrist. */
  public Wrist() {
    switch (Constants.currentMode) {
      case REAL:
        io = new WristIOTalonFX();
        io.setPID(new Slot0Configs().withKP(2.5).withKS(.125).withKV(0.067));
        io.configMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(75)
                .withMotionMagicAcceleration(1000));
        io.resetAngle(Units.degreesToRadians(-102));

        break;

      case SIM:
        io = new WristIOSim();
        io.setPID(new Slot0Configs().withKP(5));
        break;

      case REPLAY:
        io = new WristIO() {};
        break;

      default:
        System.out.println("Failed to make wrist");
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);

    if (this.getCurrentCommand() != null) {
      Logger.recordOutput("Currently Scheudled Command", this.getCurrentCommand().getName());
    }
  }

  public void resetAngle(double angle) {
    io.resetAngle(angle);
  }

  public void runPosition(double angle) {
    io.setPosition(angle);
  }

  public void runVolts(DoubleSupplier volt) {
    io.setVoltage(volt.getAsDouble());
  }

  public double getAngle() {
    return Units.degreesToRadians(inputs.positionDeg);
  }

  public Command setPosition(DoubleSupplier angle) {
    return Commands.run(() -> runPosition(angle.getAsDouble()), getInstance());
  }

  public void floppy(boolean flop) {
    io.floppy(flop);
  }

  public double getVelocityRPM() {
    return inputs.velocityRPM;
  }

  public double getAppliedVolts() {
    return inputs.appliedVolts;
  }

  public double getCurrent() {
    return inputs.torqueCurrentAmps;
  }

  public boolean getBeamBreak() {
    return inputs.beamBreak;
  }

  public void setMode(NeutralModeValue mode) {
    io.setMode(mode);
  }
}
