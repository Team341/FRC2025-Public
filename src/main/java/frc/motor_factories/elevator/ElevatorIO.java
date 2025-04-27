package frc.motor_factories.elevator;

import com.ctre.phoenix6.configs.Slot0Configs;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public class ElevatorIOInputs {

    /** Applied voltage to the elevator system in Volts. */
    public double masterAppliedVolts;

    public double masterVelocity;

    public double slaveAppliedVolts;
    /** Temperature of the elevator system in degrees Celsius. */
    public double masterTemperatureCelsius;

    public double slaveTemperatureCelsius;
    /** Current drawn by the flywheel system in Amps. */
    double masterCurrentAmps;

    double slaveCurrentAmps;

    /** The current position of the elevator carriage */
    public double positionMeters;

    /** The state of the elevators lower limit switch */
    public boolean lowerLimitSwitchHit;

    String name;

    // *********************************************************************************
    // Subsystem status check/states
    // *********************************************************************************
    public double desiredPosition;
  }

  /** Stops the elevator control by setting the velocity to zero and/or cutting off the voltage. */
  public default void stop() {}

  /**
   * Updates the current inputs for the elevaotr system.
   *
   * @param inputs The {@link ElevatorIOInputs} object containing the latest input data for the
   *     elevator, such as temperature, position, voltage, and current.
   */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /**
   * Sets the voltage to the elevator system.
   *
   * @param volts The voltage to be applied to the elevator. A positive value will spin the flywheel
   *     forward, while a negative value will spin it in reverse.
   */
  public default void setVoltage(double volts) {}

  /**
   * Sets the target position for the elevator system.
   *
   * @param position The desired position in meters.
   */
  public default void setPosition(double position) {}

  /**
   * Sets the PID and feedforward constants for controlling the elevator.
   *
   * @param kP Proportional gain constant for the PID controller.
   * @param kI Integral gain constant for the PID controller.
   * @param kD Derivative gain constant for the PID controller.
   * @param FF Feedforward constant for the flywheel control.
   * @param kS Static gain constant for the flywheel control.
   * @param kV Velocity gain constant for the flywheel control.
   */
  public default void setPID(double kP, double kI, double kD, double FF, double kS, double kV) {}
  /**
   * Sets the PID and feedforward constants for controlling the elevator.
   *
   * @param configs the Pheonix6 class for defining PID gains
   */
  public default void setPID(Slot0Configs configs) {}

  public default void resetPosition(double meters) {}

  public default void setPositionDynamic(
      double position, double velocity, double acceleration, double jerk) {}
}
