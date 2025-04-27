package frc.robot.subsystems.coral;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.motor_factories.motors.TalonFXFactory;

public class CoralIOTalonFX implements CoralIO {
  private TalonFX pivot;
  private TalonFX intake;
  DigitalInput pivotLimitSwitch;

  public CoralIOTalonFX() {
    pivot = TalonFXFactory.createDefaultTalon(CoralConstants.PIVOT_ID);
    intake = TalonFXFactory.createDefaultTalon(CoralConstants.INTAKE_ID);
    CurrentLimitsConfigs config = new CurrentLimitsConfigs();
    config.StatorCurrentLimit = 100;
    config.SupplyCurrentLimit = 100;
    config.StatorCurrentLimitEnable = true;
    config.StatorCurrentLimitEnable = true;

    intake.getConfigurator().apply(config);

    MotionMagicConfigs pivotConfig = new MotionMagicConfigs();

    pivotLimitSwitch = new DigitalInput(2);
    pivotConfig.MotionMagicCruiseVelocity = 80;
    pivotConfig.MotionMagicAcceleration = 240;
    pivot.getConfigurator().apply(pivotConfig);

    MotorOutputConfigs mOutConfigs = new MotorOutputConfigs();
    mOutConfigs.Inverted = InvertedValue.Clockwise_Positive;

    mOutConfigs.NeutralMode = NeutralModeValue.Brake;
    pivot.getConfigurator().apply(mOutConfigs);
  }

  @Override
  public void updateInputs(CoralIOInputs inputs) {
    inputs.intakeAppliedVolts = intake.getMotorVoltage().getValueAsDouble();
    inputs.intakeSupplyCurrent = intake.getSupplyCurrent().getValueAsDouble();
    inputs.intakeStatorCurrent = intake.getStatorCurrent().getValueAsDouble();
    inputs.intakeTemperature = intake.getDeviceTemp().getValueAsDouble();
    inputs.intakeVelocityRPM = intake.getVelocity().getValueAsDouble();

    inputs.pivotLimitSwitchPressed = !pivotLimitSwitch.get();

    inputs.pivotAppliedVolts = pivot.getMotorVoltage().getValueAsDouble();
    inputs.pivotSupplyCurrent = pivot.getSupplyCurrent().getValueAsDouble();
    inputs.pivotTorqueCurrent = pivot.getTorqueCurrent().getValueAsDouble();
    inputs.pivotTemperature = pivot.getDeviceTemp().getValueAsDouble();
    inputs.pivotVelocityRPM =
        (pivot.getVelocity().getValueAsDouble() / CoralConstants.PIVOT_INTAKE_RATIO) * 60;
    inputs.pivotAngleDeg =
        Units.rotationsToDegrees(
            pivot.getPosition().getValueAsDouble() / CoralConstants.PIVOT_INTAKE_RATIO);
  }

  @Override
  public void setPivotAngle(double angle) {
    pivot.setControl(
        new MotionMagicVoltage(Units.radiansToRotations(angle * CoralConstants.PIVOT_INTAKE_RATIO))
            .withEnableFOC(true));
    // pivot.setControl(new VoltageOut(0));
  }

  @Override
  public void runPivotVolts(double volts) {
    pivot.setControl(new VoltageOut(volts));
  }

  @Override
  public void stopPivot() {
    pivot.stopMotor();
  }

  @Override
  public void runIntakeVolts(double volts) {
    intake.setControl(new DutyCycleOut(volts).withEnableFOC(true));
  }

  public void setMM(MotionMagicConfigs mm) {
    pivot.getConfigurator().apply(mm);
  }

  @Override
  public void stopIntake() {
    intake.stopMotor();
  }

  @Override
  public void setPivotPID(Slot0Configs pid) {
    pivot.getConfigurator().apply(pid);
  }

  public void resetAngle(double angle) {
    pivot.setPosition(Units.radiansToRotations(angle) * CoralConstants.PIVOT_INTAKE_RATIO);
  }

  @Override
  public void setMode(NeutralModeValue x) {
    MotorOutputConfigs mOutConfigs = new MotorOutputConfigs();
    mOutConfigs.Inverted = InvertedValue.Clockwise_Positive;
    mOutConfigs.NeutralMode = x;
    pivot.getConfigurator().apply(mOutConfigs);

    MotorOutputConfigs mOutConfigs2 = new MotorOutputConfigs();
    mOutConfigs.NeutralMode = x;
    intake.getConfigurator().apply(mOutConfigs2);
  }
}
