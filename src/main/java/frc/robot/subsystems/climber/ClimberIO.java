package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public class ClimberIOInputs {
    public double appliedVolts;
    public double torqueCurrent;
    public double supplyCurrent;
    public double temperatureCelsius;
    public double velocityRPM;
    public double positionDeg;

    public double absolutePosition;
    public boolean limitSwitch;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void runAngle(double angle) {}

  public default void runVolts(double volts) {}

  public default void setGrabberVolts(double volts) {}

  public default void stop() {}

  public default void setPID(Slot0Configs config) {}

  public default void setMM(MotionMagicConfigs mm) {}

  public default void resetAngle(double angle) {}
}
