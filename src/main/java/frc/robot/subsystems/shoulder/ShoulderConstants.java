package frc.robot.subsystems.shoulder;

import edu.wpi.first.math.util.Units;
import frc.motor_factories.motors.CanDeviceId;
import frc.robot.Constants;

public class ShoulderConstants {
  public static final CanDeviceId SHOULDER_ID = new CanDeviceId(6, Constants.CANIVORE_NAME);
  public static final CanDeviceId ENCODER_ID = new CanDeviceId(28, Constants.CANIVORE_NAME);
  public static final double SHOULDER_LENGTH = Units.inchesToMeters(21.5);
}
