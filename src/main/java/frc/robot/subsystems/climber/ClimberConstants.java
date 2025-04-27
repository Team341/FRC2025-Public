package frc.robot.subsystems.climber;

import frc.motor_factories.motors.CanDeviceId;

public class ClimberConstants {
  public static final CanDeviceId CLIMBER_ID = new CanDeviceId(16);
  public static final double CLIMBER_LENGTH = 0.8;
  public static final int DIO_PORT = 4;
  public static final double CLIMBER_POST_ANGLE = 77;
  public static final double CLIMBER_K_DAMPENING_P = 0.001;
}
