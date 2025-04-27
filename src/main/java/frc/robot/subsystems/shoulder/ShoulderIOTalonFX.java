package frc.robot.subsystems.shoulder;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import frc.motor_factories.motors.TalonFXFactory;
import java.util.function.DoubleSupplier;

public class ShoulderIOTalonFX implements ShoulderIO {
  TalonFX shoulder;
  CANcoder encoder;
  private StatusSignal<Angle> shoulderPos;

  public ShoulderIOTalonFX() {
    shoulder = TalonFXFactory.createDefaultTalon(ShoulderConstants.SHOULDER_ID);
    encoder = new CANcoder(28, "CANIVORE 3");
    MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
    outputConfigs.DutyCycleNeutralDeadband = 0.0;
    outputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    outputConfigs.NeutralMode = NeutralModeValue.Brake;
    shoulder.getConfigurator().apply(outputConfigs);

    CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
    cc_cfg.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(.5);
    cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    cc_cfg.MagnetSensor.withMagnetOffset(0.415039);
    encoder.getConfigurator().apply(cc_cfg);

    SoftwareLimitSwitchConfigs softLimitConfigs = new SoftwareLimitSwitchConfigs();
    softLimitConfigs.ForwardSoftLimitEnable = true;
    softLimitConfigs.ReverseSoftLimitEnable = true;
    softLimitConfigs.ForwardSoftLimitThreshold = 0.13;
    softLimitConfigs.ReverseSoftLimitThreshold = -0.69;

    FeedbackConfigs shoulderFeedbackConfigs = new FeedbackConfigs();
    shoulderFeedbackConfigs.FeedbackRemoteSensorID = encoder.getDeviceID();
    shoulderFeedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    shoulderFeedbackConfigs.SensorToMechanismRatio = 1.0;
    shoulderFeedbackConfigs.RotorToSensorRatio = 45.0;
    shoulder.getConfigurator().apply(shoulderFeedbackConfigs);

    shoulderPos = shoulder.getPosition();
    shoulderPos.setUpdateFrequency(500);
  }

  @Override
  public void updateInputs(ShoulderIOInputs inputs) {
    inputs.absolutePosition = encoder.getAbsolutePosition().getValueAsDouble();
    inputs.appliedVolts = shoulder.getMotorVoltage().getValueAsDouble();
    inputs.positionDeg = (Units.rotationsToDegrees(shoulder.getPosition().getValueAsDouble()) - 90);
    inputs.supplyCurrent = shoulder.getSupplyCurrent().getValueAsDouble();
    inputs.torqueCurrent = shoulder.getTorqueCurrent().getValueAsDouble();
    inputs.velocityRPM = shoulder.getVelocity().getValueAsDouble() * 60;
    inputs.temperatureCelsius = shoulder.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void stop() {
    shoulder.stopMotor();
  }

  @Override
  public void runAngle(double angle) {
    shoulder.setControl(
        new MotionMagicVoltage(Units.radiansToRotations(angle + Math.PI / 2.)).withEnableFOC(true));
  }

  @Override
  public void runAngleDynamic(double angle, double velo, double accel, double jerk) {
    shoulder.setControl(
        new DynamicMotionMagicVoltage(
                Units.radiansToRotations(angle + Math.PI / 2), velo, accel, jerk)
            .withEnableFOC(true));
  }

  @Override
  public void resetAngle(double angle) {
    shoulder.setPosition(Units.radiansToRotations(angle));
  }

  @Override
  public void runVolts(double volts) {
    shoulder.setControl(new VoltageOut(volts));
  }

  @Override
  public void setPID(Slot0Configs pidConfig) {
    pidConfig.GravityType = GravityTypeValue.Arm_Cosine;
    shoulder.getConfigurator().apply(pidConfig);
  }

  @Override
  public void setMM(MotionMagicConfigs mm) {
    shoulder.getConfigurator().apply(mm);
  }

  @Override
  public void setMode(NeutralModeValue mode) {
    MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
    outputConfigs.DutyCycleNeutralDeadband = 0.0;
    outputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    outputConfigs.NeutralMode = mode;
    shoulder.getConfigurator().apply(outputConfigs);
  }

  @Override
  public DoubleSupplier getShoulderSupplier() {
    return () -> (Units.rotationsToDegrees(shoulderPos.getValueAsDouble()) - 90);
  }
}
