package frc.robot.subsystems.shoulder;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import java.util.function.DoubleSupplier;

public class ShoulderIOSim implements ShoulderIO {
  private SingleJointedArmSim shoulder;
  private boolean isClosedLoop;
  private double appliedVolts;
  private PIDController pid;

  public ShoulderIOSim() {
    shoulder =
        new SingleJointedArmSim(
            DCMotor.getFalcon500(1),
            10.0,
            .003,
            .8,
            Units.degreesToRadians(-160),
            Units.degreesToRadians(160),
            false,
            0);
    pid = new PIDController(1.0, 0, 0);
  }

  @Override
  public void updateInputs(ShoulderIOInputs inputs) {
    if (isClosedLoop) appliedVolts = pid.calculate(shoulder.getAngleRads());
    shoulder.setInputVoltage(appliedVolts);
    shoulder.update(0.02);

    inputs.absolutePosition = 0.0;

    inputs.appliedVolts = appliedVolts;
    inputs.positionDeg = Units.radiansToDegrees(shoulder.getAngleRads());
    inputs.supplyCurrent = shoulder.getCurrentDrawAmps();
    inputs.torqueCurrent = 0.0;
    inputs.temperatureCelsius = 0.0;
    inputs.velocityRPM =
        Units.radiansPerSecondToRotationsPerMinute(shoulder.getVelocityRadPerSec());
  }

  @Override
  public void stop() {
    appliedVolts = 0.0;
    isClosedLoop = false;
  }

  @Override
  public void runAngle(double angle) {
    isClosedLoop = true;
    pid.setSetpoint((angle));
  }

  @Override
  public void runAngleDynamic(double angle, double velo, double accel, double jerk) {
    isClosedLoop = true;
    pid.setSetpoint((angle));
  }

  @Override
  public void runVolts(double volts) {
    appliedVolts = volts;
    isClosedLoop = false;
  }

  @Override
  public void setPID(Slot0Configs config) {
    pid.setP(config.kP);
    pid.setI(config.kI);
    pid.setD(config.kD);
  }

  @Override
  public void setMM(MotionMagicConfigs mmConfig) {}

  @Override
  public void resetAngle(double angle) {
    shoulder.setState(angle, shoulder.getVelocityRadPerSec());
  }

  @Override
  public DoubleSupplier getShoulderSupplier() {
    return () -> Units.radiansToDegrees(shoulder.getAngleRads());
  }
}
