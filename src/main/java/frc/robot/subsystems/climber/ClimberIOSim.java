package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ClimberIOSim implements ClimberIO {
  private SingleJointedArmSim climber;
  private boolean isClosedLoop;
  private double appliedVolts;
  private PIDController pid;

  public ClimberIOSim() {
    climber =
        new SingleJointedArmSim(
            DCMotor.getFalcon500(1),
            10.0,
            .3,
            .8,
            Units.degreesToRadians(-160),
            Units.degreesToRadians(160),
            false,
            0);
    pid = new PIDController(1.0, 0, 0);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    if (isClosedLoop) appliedVolts = pid.calculate(climber.getAngleRads());
    climber.setInputVoltage(appliedVolts);
    climber.update(0.02);

    inputs.absolutePosition = 0.0;

    inputs.appliedVolts = appliedVolts;
    inputs.positionDeg = Units.radiansToDegrees(climber.getAngleRads());
    inputs.supplyCurrent = climber.getCurrentDrawAmps();
    inputs.torqueCurrent = 0.0;
    inputs.temperatureCelsius = 0.0;
    inputs.velocityRPM = Units.radiansPerSecondToRotationsPerMinute(climber.getVelocityRadPerSec());
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
    climber.setState(angle, climber.getVelocityRadPerSec());
  }
}
