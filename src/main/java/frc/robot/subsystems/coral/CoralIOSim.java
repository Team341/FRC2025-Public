package frc.robot.subsystems.coral;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.littletonrobotics.junction.Logger;

public class CoralIOSim implements CoralIO {
  private SingleJointedArmSim pivot;
  private FlywheelSim intake;
  private PIDController pid;

  boolean isClosedLoop = false;
  double appliedVolts = 0.;

  public CoralIOSim() {
    pivot =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            5,
            .2,
            Units.inchesToMeters(16),
            0,
            Math.PI / 2,
            false,
            Math.PI / 2);
    intake =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.001, 1), DCMotor.getNEO(1));
    pid = new PIDController(0, 0, 0);
  }

  @Override
  public void updateInputs(CoralIOInputs inputs) {
    if (isClosedLoop) {
      appliedVolts = pid.calculate(pivot.getAngleRads());
    }

    pivot.setInputVoltage(appliedVolts);

    pivot.update(0.02);
    intake.update(0.02);

    inputs.pivotAppliedVolts = appliedVolts;
    inputs.pivotSupplyCurrent = pivot.getCurrentDrawAmps();
    inputs.pivotTorqueCurrent = 0.0;
    inputs.pivotVelocityRPM = pivot.getVelocityRadPerSec() * 60 / 2 * Math.PI;
    inputs.pivotTemperature = 0.0;
    inputs.pivotAngleDeg = Units.radiansToDegrees(pivot.getAngleRads());

    inputs.intakeAppliedVolts = intake.getInputVoltage();
    inputs.intakeSupplyCurrent = intake.getCurrentDrawAmps();
    inputs.intakeVelocityRPM = intake.getAngularVelocityRPM();
    inputs.intakeTemperature = 0.0;

    Logger.recordOutput("Pivot/isClosedLoop", isClosedLoop);
    Logger.recordOutput("Pivot/kP", pid.getP());
  }

  @Override
  public void setPivotAngle(double angle) {
    isClosedLoop = true;
    pid.setSetpoint((angle));
  }

  @Override
  public void runPivotVolts(double voltage) {
    isClosedLoop = false;
    appliedVolts = voltage;
  }

  @Override
  public void stopPivot() {
    isClosedLoop = false;
    appliedVolts = 0;
  }

  @Override
  public void runIntakeVolts(double volts) {
    intake.setInputVoltage(volts);
  }

  @Override
  public void stopIntake() {
    intake.setInputVoltage(0);
  }

  @Override
  public void setPivotPID(Slot0Configs config) {
    pid.setP(config.kP);
    pid.setI(config.kI);
    pid.setD(config.kD);
  }

  public void resetAngle(double angle) {
    pivot.setState(angle, pivot.getVelocityRadPerSec());
  }
}
