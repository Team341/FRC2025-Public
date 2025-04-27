package frc.robot.subsystems.gripper;

import com.ctre.phoenix6.signals.NeutralModeValue;
import org.littletonrobotics.junction.AutoLog;

public interface GripperIO {
  @AutoLog
  public class GripperIOInputs {
    public boolean beamBreak;

    public double velocityRPM;
    public double appliedVolts;
    public double torqueCurrent;
    public double supplyCurrent;
    public double temperatureCelsius;

    public boolean flingMode;
    public boolean notifierCalled;
  }

  public default void updateInputs(GripperIOInputs inputs) {}

  public default void runVolts(double volts) {}

  public default void stop() {}

  public default void setMode(NeutralModeValue mode) {}

  public default void setFling(boolean fling) {}
}
