package frc.motor_factories.elevator;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
  private ElevatorSim sim;
  private boolean isClosedLoop = false;
  private double appliedVolts = 0.0;
  private ElevatorDescription modelDescription = null;
  private String name;
  private PIDController pid = new PIDController(0.0, 0.0, 0.0);
  private double desiredPositon;

  public ElevatorIOSim(String name, ElevatorDescription model) {
    this.name = name;
    this.modelDescription = model;

    DCMotor motor = DCMotor.getKrakenX60(2);

    sim =
        new ElevatorSim(
            LinearSystemId.createElevatorSystem(
                motor, model.getMassKg(), model.getDrumRadiusInM(), model.getGearing()),
            motor,
            model.getMinHeight(),
            model.getMaxHeight(),
            true,
            model.getStartingHeight());

    setPID(model.getSimPidGains());
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    if (isClosedLoop) {
      appliedVolts = pid.calculate(sim.getPositionMeters());
      sim.setInputVoltage(appliedVolts);
    }

    sim.update(0.02);
    inputs.masterAppliedVolts = appliedVolts;
    inputs.masterCurrentAmps = sim.getCurrentDrawAmps();
    inputs.positionMeters = sim.getPositionMeters();
    inputs.lowerLimitSwitchHit = inputs.positionMeters <= this.modelDescription.getMinHeight();
    inputs.masterTemperatureCelsius = 0.0;
    inputs.desiredPosition = desiredPositon;
  }

  @Override
  public void stop() {
    isClosedLoop = false;
    appliedVolts = 0.0;
    sim.setInputVoltage(appliedVolts);
  }

  @Override
  public void setPosition(double distance) {
    isClosedLoop = true;
    pid.setSetpoint(distance);
    desiredPositon = distance;
  }

  @Override
  public void setPositionDynamic(double distance, double s, double d, double f) {
    isClosedLoop = true;
    pid.setSetpoint(distance);
    desiredPositon = distance;
  }

  @Override
  public void setVoltage(double volts) {
    isClosedLoop = false;
    appliedVolts = volts;
    sim.setInputVoltage(appliedVolts);
  }

  @Override
  public void setPID(double kP, double kI, double kD, double FF, double kS, double kV) {
    pid.setP(kP);
    pid.setI(kI);
    pid.setD(kD);
  }

  public void setPID(Slot0Configs config) {
    this.setPID(config.kP, config.kI, config.kD, config.kG, config.kS, config.kV);
  }

  @Override
  public void resetPosition(double meters) {
    sim.setState(meters, sim.getVelocityMetersPerSecond());
  }
}
