package frc.robot.subsystems.gripper;

import frc.motor_factories.motors.CanDeviceId;
import frc.robot.Constants;
import lombok.Getter;

public class GripperConstants {
  @Getter private static final CanDeviceId gripperID = new CanDeviceId(4, Constants.CANIVORE_NAME);
  @Getter private static final int beamBreakID = 0;
}
