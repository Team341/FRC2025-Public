package frc.robot.subsystems.wrist;

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
import frc.motor_factories.motors.CanDeviceId;
import frc.motor_factories.motors.TalonFXFactory;
import frc.robot.Constants;

public class WristIOTalonFX implements WristIO {
  private TalonFX wrist;
  private DigitalInput beambreak;
  private final int ID = 5;
  private final double GEAR_RATIO = 21.3889;
  private final int DIO_ID = 1;

  public WristIOTalonFX() {
    wrist = TalonFXFactory.createDefaultTalon(new CanDeviceId(ID, Constants.CANIVORE_NAME));
    beambreak = new DigitalInput(DIO_ID);

    MotorOutputConfigs config = new MotorOutputConfigs();
    config.Inverted = InvertedValue.Clockwise_Positive;
    config.NeutralMode = NeutralModeValue.Brake;
    wrist.getConfigurator().apply(config);
  }

  public void updateInputs(WristIOInputs inputs) {
    inputs.appliedVolts = wrist.getMotorVoltage().getValueAsDouble();
    inputs.positionDeg =
        Units.rotationsToDegrees(wrist.getPosition().getValueAsDouble() / GEAR_RATIO);
    inputs.supplyCurrentAmps = wrist.getSupplyCurrent().getValueAsDouble();
    inputs.torqueCurrentAmps = wrist.getTorqueCurrent().getValueAsDouble();
    inputs.velocityRPM =
        Units.rotationsToDegrees(wrist.getVelocity().getValueAsDouble() / GEAR_RATIO) * 60;
    inputs.beamBreak = beambreak.get();
  }

  @Override
  public void setVoltage(double volts) {
    wrist.setControl(new VoltageOut(volts));
  }

  @Override
  public void setPosition(double position) {
    wrist.setControl(
        new MotionMagicVoltage(Units.radiansToRotations(position) * GEAR_RATIO)
            .withEnableFOC(true));
  }

  @Override
  public void setPID(Slot0Configs pidConfig) {
    wrist.getConfigurator().apply(pidConfig);
  }

  @Override
  public void resetAngle(double angle) {
    wrist.setPosition(Units.radiansToRotations(angle) * GEAR_RATIO);
  }

  @Override
  public void floppy(boolean flop) {
    if (flop) {
      wrist.stopMotor();
      wrist.setNeutralMode(NeutralModeValue.Coast);
    } else {
      wrist.setNeutralMode(NeutralModeValue.Brake);
    }
  }

  @Override
  public void configMotionMagic(MotionMagicConfigs mmConfig) {
    wrist.getConfigurator().apply(mmConfig);
  }

  @Override
  public void setMode(NeutralModeValue mode) {
    MotorOutputConfigs config = new MotorOutputConfigs();
    config.Inverted = InvertedValue.Clockwise_Positive;
    config.NeutralMode = mode;
    wrist.getConfigurator().apply(config);
  }
}
