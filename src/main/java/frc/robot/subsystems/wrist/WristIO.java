package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    public double appliedVolts;
    public double torqueCurrentAmps;
    public double supplyCurrentAmps;
    public double positionDeg;
    public double temperatureCelsius;
    public double velocityRPM;

    public boolean beamBreak;
  }

  public default void updateInputs(WristIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setPosition(double position) {}

  public default void setPID(Slot0Configs pid) {}

  public default void configMotionMagic(MotionMagicConfigs mm) {}

  public default void resetAngle(double angle) {}

  public default void floppy(boolean flop) {}

  public default void setMode(NeutralModeValue mode) {}
}
