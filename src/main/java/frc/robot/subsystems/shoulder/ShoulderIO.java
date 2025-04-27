package frc.robot.subsystems.shoulder;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLog;

public interface ShoulderIO {
  @AutoLog
  public class ShoulderIOInputs {
    public double appliedVolts;
    public double torqueCurrent;
    public double supplyCurrent;
    public double temperatureCelsius;
    public double velocityRPM;
    public double positionDeg;

    public double absolutePosition;
  }

  public default void updateInputs(ShoulderIOInputs inputs) {}

  public default void runAngle(double angle) {}

  public default void runVolts(double volts) {}

  public default void stop() {}

  public default void setPID(Slot0Configs config) {}

  public default void setMM(MotionMagicConfigs mm) {}

  public default void resetAngle(double angle) {}

  public default void runAngleDynamic(double angle, double velo, double accel, double jerk) {}

  public default void setMode(NeutralModeValue mode) {}

  public default DoubleSupplier getShoulderSupplier() {
    return null;
  }
  ;
}
