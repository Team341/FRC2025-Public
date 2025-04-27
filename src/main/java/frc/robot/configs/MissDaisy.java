package frc.robot.configs;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.util.Units;
import frc.motor_factories.elevator.ElevatorDescription;
import frc.motor_factories.motors.CanDeviceId;
import java.util.Map;

public class MissDaisy implements RobotConstants {

  Map<String, CanDeviceId> shooterMotors = null;
  ElevatorDescription elevatorDescription = null;

  public MissDaisy() {
    elevatorDescription =
        new ElevatorDescription()
            .withDrumRadius(Units.inchesToMeters(0.75))
            .addMasterMotor(new CanDeviceId(2, CanivoreName))
            .addSlaveMotor(new CanDeviceId(3, CanivoreName))
            .withFollowInverted(true)
            .withGearingToDrum(3)
            .withLowerLimitSwitch(0)
            .withMassInKg(.02)
            .withMaxHeight(Units.inchesToMeters(51))
            .withMinHeight(0)
            .withOutputToHeight(1)
            .withPIDGains(
                new Slot0Configs()
                    .withKP(3)
                    .withKG(0.4)
                    .withKS(0.55)
                    // .withKV(0.1241)
                    // .withKA(0.0016)
                    .withGravityType(GravityTypeValue.Elevator_Static))
            .withSimulationPIDGains(new Slot0Configs().withKP(15))
            .withStartingHeight(0)
            .withMotionMagicConfigs(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(85)
                    .withMotionMagicAcceleration(185)); // 250
  }

  public Map<String, CanDeviceId> getShooterMotors() {
    return shooterMotors;
  }

  public ElevatorDescription getElevatorDescription() {
    return elevatorDescription;
  }
}
