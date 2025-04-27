package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class WristIOSim implements WristIO {
  private boolean isClosedLoop = false;
  private PIDController pid = new PIDController(1.0, 0, 0);
  private SingleJointedArmSim sim;
  private double appliedVolts = 0.0;

  private final double GEARING = 20.0;
  private final double MOI = 0.01;
  private final double ARM_LENGTH = Units.inchesToMeters(22);
  private final double MIN_ANGLE = Units.degreesToRadians(-90);
  private final double MAX_ANGLE = Units.degreesToRadians(90);

  public WristIOSim() {
    sim =
        new SingleJointedArmSim(
            DCMotor.getFalcon500(1),
            GEARING,
            MOI,
            ARM_LENGTH,
            MIN_ANGLE,
            MAX_ANGLE,
            false,
            Units.degreesToRadians(0));
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    if (isClosedLoop) appliedVolts = pid.calculate(sim.getAngleRads());

    sim.setInputVoltage(appliedVolts);
    sim.update(0.02);

    inputs.temperatureCelsius = 0.0;
    inputs.positionDeg = Units.radiansToDegrees(sim.getAngleRads());
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.velocityRPM = Units.radiansPerSecondToRotationsPerMinute(sim.getVelocityRadPerSec());
    inputs.torqueCurrentAmps = 0.0;
    inputs.appliedVolts = appliedVolts;
  }

  @Override
  public void setVoltage(double volts) {
    isClosedLoop = false;
    appliedVolts = volts;
  }

  @Override
  public void setPosition(double position) {
    isClosedLoop = true;
    pid.setSetpoint((position));
  }

  @Override
  public void setPID(Slot0Configs config) {
    pid.setP(config.kP);
    pid.setI(config.kI);
    pid.setD(config.kD);
  }

  @Override
  public void floppy(boolean flop) {

    isClosedLoop = false;
    appliedVolts = 0;
  }

  @Override
  public void resetAngle(double angle) {
    sim.setState(angle, sim.getVelocityRadPerSec());
  }
}
