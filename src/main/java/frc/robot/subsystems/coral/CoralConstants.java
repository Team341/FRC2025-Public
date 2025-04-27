package frc.robot.subsystems.coral;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.util.Units;
import frc.motor_factories.motors.CanDeviceId;

public class CoralConstants {
  public static final CanDeviceId INTAKE_ID = new CanDeviceId(10);
  public static final CanDeviceId PIVOT_ID = new CanDeviceId(11);
  public static final double PIVOT_INTAKE_RATIO = 63;
  public static final Slot0Configs REAL_PID_CONFIGS = new Slot0Configs().withKP(10.).withKS(0.1);
  public static final Slot0Configs SIM_PID_CONFIGS = new Slot0Configs().withKP(5);
  public static final double APPROXIMATE_LENGTH = Units.inchesToMeters(17);
  public static final double APPROXIMATE_X_ORIGIN = -Units.inchesToMeters(9.25);
}
