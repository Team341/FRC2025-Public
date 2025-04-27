// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shoulder;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Shoulder extends SubsystemBase {
  private ShoulderIOInputsAutoLogged inputs = new ShoulderIOInputsAutoLogged();
  private ShoulderIO io;
  private DoubleSupplier shoulderAngleSupplier;

  private static Shoulder instance;

  public static Shoulder getInstance() {
    if (instance == null) return instance = new Shoulder();
    return instance;
  }

  /** Creates a new Shoulder. */
  public Shoulder() {

    switch (Constants.currentMode) {
      case REAL:
        io = new ShoulderIOTalonFX();
        io.setMM(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(2)
                .withMotionMagicAcceleration(9.));
        io.setPID(
            new Slot0Configs()
                .withKP(65)
                .withKG(0.45)
                .withKS(0.2375)
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign));
        shoulderAngleSupplier = io.getShoulderSupplier();

        break;
      case SIM:
        io = new ShoulderIOSim();
        io.setPID(new Slot0Configs().withKP(1.5));
        shoulderAngleSupplier = io.getShoulderSupplier();
        break;
      case REPLAY:
        io = new ShoulderIO() {};
        break;
      default:
        System.out.println("Shoulder not created properly");
        break;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Shoulder", inputs);
  }

  public void halt() {
    io.stop();
  }

  public void runAngle(double angle) {
    io.runAngle(angle);
  }

  public void runAngleDynamic(double angle, Supplier<MotionMagicConfigs> config) {
    io.runAngleDynamic(
        angle,
        config.get().MotionMagicCruiseVelocity,
        config.get().MotionMagicAcceleration,
        config.get().MotionMagicJerk);
  }

  public void runVolts(DoubleSupplier volts) {
    io.runVolts(volts.getAsDouble());
  }

  public void resetAngle(double angle) {
    io.resetAngle(angle);
  }

  public Command setAngle(DoubleSupplier angle) {
    return Commands.run(
        () -> {
          runAngle(angle.getAsDouble());
        },
        instance);
  }

  public Command setVolts(double volts) {
    return Commands.run(() -> io.runVolts(volts), instance);
  }

  public double getAngle() {
    return Units.degreesToRadians(inputs.positionDeg);
  }

  public double getVelocityDegPerSec() {
    return Units.rotationsToDegrees(inputs.velocityRPM) / 60;
  }

  public void setMode(NeutralModeValue mode) {
    io.setMode(mode);
  }

  public DoubleSupplier getAngleSupplier() {
    return shoulderAngleSupplier;
  }
}
