package frc.robot.subsystems.coral;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.littletonrobotics.junction.AutoLog;

public interface CoralIO {
  @AutoLog
  public class CoralIOInputs {
    public double pivotAppliedVolts;
    public double pivotTemperature;
    public double pivotVelocityRPM;
    public double pivotSupplyCurrent;
    public double pivotTorqueCurrent;
    public double pivotAngleDeg;

    public double intakeTemperature;
    public double intakeVelocityRPM;
    public double intakeSupplyCurrent;
    public double intakeStatorCurrent;
    public double intakeAppliedVolts;
    public boolean pivotLimitSwitchPressed;
  }

  public default void updateInputs(CoralIOInputs inputs) {}

  public default void setPivotAngle(double angle) {}

  public default void stopPivot() {}

  public default void runPivotVolts(double volts) {}

  public default void runIntakeVolts(double volts) {}

  public default void stopIntake() {}

  public default void setPivotPID(Slot0Configs pid) {}

  public default void resetAngle(double angle) {}

  public default void setMode(NeutralModeValue mode) {}
}
