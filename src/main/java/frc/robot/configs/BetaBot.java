package frc.robot.configs;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.util.Units;
import frc.motor_factories.elevator.ElevatorDescription;
import frc.motor_factories.motors.CanDeviceId;
import java.util.Map;

public class BetaBot implements RobotConstants {

  Map<String, CanDeviceId> shooterMotors = null;
  ElevatorDescription elevatorDescription = null;

  public BetaBot() {
    elevatorDescription =
        new ElevatorDescription()
            .withDrumRadius(Units.inchesToMeters(1.1 / 2))
            .addMasterMotor(new CanDeviceId(3, CanivoreName))
            .addSlaveMotor(new CanDeviceId(4, CanivoreName))
            .withFollowInverted(false)
            .withGearingToDrum(2.90909)
            .withLowerLimitSwitch(1)
            .withMassInKg(.02)
            .withMaxHeight(Units.inchesToMeters(51))
            .withMinHeight(0)
            .withOutputToHeight(1)
            .withPIDGains(
                new Slot0Configs()
                    .withKP(1)
                    .withKG(0.6)
                    .withGravityType(GravityTypeValue.Elevator_Static))
            .withSimulationPIDGains(new Slot0Configs().withKP(15))
            .withStartingHeight(0)
            .withMotionMagicConfigs(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(60)
                    .withMotionMagicAcceleration(100));
  }

  public Map<String, CanDeviceId> getShooterMotors() {
    return shooterMotors;
  }

  public ElevatorDescription getElevatorDescription() {
    return elevatorDescription;
  }
}
