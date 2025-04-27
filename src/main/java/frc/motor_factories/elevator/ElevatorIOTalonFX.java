package frc.motor_factories.elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import frc.motor_factories.motors.TalonFXFactory;

public class ElevatorIOTalonFX implements ElevatorIO {
  private ElevatorDescription description;
  private TalonFX masterMotor;
  private TalonFX slaveMotor;
  private String name;
  // private DigitalInput lowerLimit;

  public ElevatorIOTalonFX(ElevatorDescription description, String name) {
    this.name = name;
    this.description = description;

    //

    // Create the master motor based on the first item in the list of motors in the
    // description
    masterMotor = TalonFXFactory.createDefaultTalon(description.getMasterMotor());

    SetMotorDirection(masterMotor, InvertedValue.Clockwise_Positive);
    slaveMotor =
        TalonFXFactory.createPermanentFollowerTalon(
            description.getSlaveMotor(), description.getMasterMotor(), true);

    // lowerLimit = new DigitalInput(description.getLimitSwitchPort());

    setPID(description.getPidGains());
    setMM(description.getMotionMagicConfigs());

    SoftwareLimitSwitchConfigs softwareLimit = new SoftwareLimitSwitchConfigs();
    softwareLimit.ForwardSoftLimitEnable = true;
    softwareLimit.ForwardSoftLimitThreshold = 30.9; // 30.18
    masterMotor.getConfigurator().apply(softwareLimit);
  }

  public void SetMotorDirection(TalonFX motor, InvertedValue direction) {
    // Copy over the inversion setting from the description
    MotorOutputConfigs config = new MotorOutputConfigs();
    config.Inverted = direction;
    motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {

    // For now, lets see if just logging the master motor is sufficient
    inputs.masterAppliedVolts = masterMotor.getMotorVoltage().getValueAsDouble();
    inputs.masterCurrentAmps = masterMotor.getStatorCurrent().getValueAsDouble();
    inputs.masterTemperatureCelsius = masterMotor.getDeviceTemp().getValueAsDouble();
    inputs.masterVelocity =
        masterMotor.getVelocity().getValueAsDouble()
            * (Math.PI * description.getDrumRadiusInM() * 2)
            / description.getGearing()
            * 60;

    inputs.slaveAppliedVolts = slaveMotor.getMotorVoltage().getValueAsDouble();
    inputs.slaveCurrentAmps = slaveMotor.getStatorCurrent().getValueAsDouble();
    inputs.slaveTemperatureCelsius = slaveMotor.getDeviceTemp().getValueAsDouble();

    // inputs.lowerLimitSwitchHit = !lowerLimit.get();
    inputs.lowerLimitSwitchHit = false;

    inputs.name = name;
    inputs.desiredPosition = masterMotor.getClosedLoopReference().getValueAsDouble();
    inputs.positionMeters =
        masterMotor.getPosition().getValueAsDouble()
            * ((Math.PI * description.getDrumRadiusInM() * 2) / description.getGearing());

    inputs.name = name;
  }

  @Override
  public void stop() {
    masterMotor.set(0);
  }

  @Override
  public void resetPosition(double meters) {
    masterMotor.setPosition(
        meters * description.getGearing() / (Math.PI * description.getDrumRadiusInM() * 2));
  }

  @Override
  public void setPosition(double distance) {

    masterMotor.setControl(
        new MotionMagicVoltage(
                (distance)
                    * description.getGearing()
                    / (Math.PI * description.getDrumRadiusInM() * 2))
            .withEnableFOC(true));
  }

  @Override
  public void setVoltage(double volts) {
    masterMotor.setControl((new VoltageOut(volts)));
  }

  @Override
  public void setPID(double kP, double kI, double kD, double FF, double kS, double kV) {
    var config = new Slot0Configs();
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;
    config.kS = kS;
    config.kV = kV;
    this.setPID(config);
  }

  @Override
  public void setPositionDynamic(double height, double velocity, double acceleration, double jerk) {
    masterMotor.setControl(
        new DynamicMotionMagicVoltage(
            (height) * description.getGearing() / (Math.PI * description.getDrumRadiusInM() * 2),
            velocity,
            acceleration,
            jerk));
  }

  public void setPID(Slot0Configs config) {
    masterMotor.getConfigurator().apply(config);
  }

  public void setMM(MotionMagicConfigs config) {
    masterMotor.getConfigurator().apply(config);
  }
}
