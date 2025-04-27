package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.motor_factories.motors.TalonFXFactory;

public class ClimberIOTalonFX implements ClimberIO {
  TalonFX climber;
  double climberGearRatio = 687.5;
  // SparkMax grabber;
  DigitalInput zeroLimit;

  public ClimberIOTalonFX() {

    climber = TalonFXFactory.createDefaultTalon(ClimberConstants.CLIMBER_ID);
    zeroLimit = new DigitalInput(ClimberConstants.DIO_PORT);

    MotorOutputConfigs outputConfigs = new MotorOutputConfigs();

    outputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    outputConfigs.NeutralMode = NeutralModeValue.Brake;

    climber.getConfigurator().apply(outputConfigs);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {

    inputs.appliedVolts = climber.getMotorVoltage().getValueAsDouble();
    inputs.positionDeg =
        Units.rotationsToDegrees(climber.getPosition().getValueAsDouble()) / climberGearRatio;
    inputs.supplyCurrent = climber.getSupplyCurrent().getValueAsDouble();
    inputs.torqueCurrent = climber.getTorqueCurrent().getValueAsDouble();
    inputs.velocityRPM = climber.getVelocity().getValueAsDouble() * 60;
    inputs.temperatureCelsius = climber.getDeviceTemp().getValueAsDouble();
    inputs.limitSwitch = !zeroLimit.get();
  }

  @Override
  public void stop() {
    climber.stopMotor();
  }

  @Override
  public void runAngle(double angle) {
    climber.setControl(
        new MotionMagicVoltage(Units.degreesToRotations(angle) * climberGearRatio)
            .withEnableFOC(true));
  }

  @Override
  public void runVolts(double volts) {
    climber.setControl(new VoltageOut(volts));
  }

  @Override
  public void resetAngle(double angle) {
    climber.setPosition(Units.radiansToRotations(angle));
  }

  @Override
  public void setPID(Slot0Configs pidConfig) {
    climber.getConfigurator().apply(pidConfig);
  }

  @Override
  public void setMM(MotionMagicConfigs mm) {
    climber.getConfigurator().apply(mm);
  }
}
