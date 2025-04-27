package frc.robot.configs;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.util.Units;
import frc.motor_factories.elevator.ElevatorDescription;
import frc.motor_factories.motors.CanDeviceId;
import java.util.Map;

public class AlphaBot implements RobotConstants {
  String CanivoreName = "CANivore";
  Map<String, CanDeviceId> shooterMotors = null;
  ElevatorDescription elevatorDescrip = null;
  double shooterGearing = 1.;

  public AlphaBot() {

    elevatorDescrip =
        new ElevatorDescription()
            .addMasterMotor(new CanDeviceId(12, CanivoreName))
            .addSlaveMotor(new CanDeviceId(13, CanivoreName))
            .withPIDGains(new Slot0Configs().withKP(0.).withKS(0.0).withKV(0.))
            .withSimulationPIDGains(new Slot0Configs().withKP(12).withKS(0.1).withKV(0.25))
            .withMinHeight(0.)
            .withMaxHeight(Units.inchesToMeters(42.5))
            .withGearingToDrum(10.)
            .withDrumRadius(Units.inchesToMeters(1.))
            .withWidth(Units.inchesToMeters(24.))
            .withOffsetX(Units.inchesToMeters(0.))
            .withSlantAngle(90.);
  }

  public Map<String, CanDeviceId> getShooterMotors() {
    return shooterMotors;
  }

  public ElevatorDescription getElevatorDescription() {
    return elevatorDescrip;
  }
}
