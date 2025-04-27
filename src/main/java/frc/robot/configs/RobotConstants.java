package frc.robot.configs;

import frc.motor_factories.elevator.ElevatorDescription;
import frc.motor_factories.motors.CanDeviceId;
import frc.robot.Constants;
import java.util.Map;

public interface RobotConstants {

  String CanivoreName = "CANIVORE 3";

  Map<String, CanDeviceId> getShooterMotors();

  ElevatorDescription getElevatorDescription();

  double shooterGearing = 1.;

  public static RobotConstants getRobotConstants(Constants.RobotIdentity robot) {
    switch (robot) {
      case KITBOT:
        return new AlphaBot();
      case BETABOT:
        return new BetaBot();
      case MISS_DAISY:
        return new MissDaisy();
      default:

        // Something went wrong if this branch is reached, by default we will return our Comp Bot
        return new MissDaisy();
    }
  }
}
