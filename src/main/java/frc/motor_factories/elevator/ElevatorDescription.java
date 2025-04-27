// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.motor_factories.elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.util.Units;
import frc.motor_factories.motors.CanDeviceId;
import lombok.Getter;

/** Add your docs here. */
public class ElevatorDescription {
  // Hardware specifications
  @Getter private CanDeviceId masterMotor;
  @Getter private CanDeviceId slaveMotor;

  @Getter private double gearing = 1;
  @Getter private double drumRadiusInM = Units.inchesToMeters(1); // meters

  @Getter private double outputToHeight = 1. / (Math.PI * Units.inchesToMeters(1.28)) * 3.33;
  @Getter private int limitSwitchPort = 5;
  // Control Parameters
  @Getter private Slot0Configs pidGains = new Slot0Configs();
  @Getter private Slot0Configs simPidGains = new Slot0Configs();
  @Getter private boolean followInverted = false;
  @Getter private MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
  // Simulation Parameters
  @Getter private double minHeight = 0.; // meters
  @Getter private double maxHeight = Units.inchesToMeters(48); // meters
  @Getter private double startingHeight = minHeight; // meters
  @Getter private double massKg = 1.; // kg

  // Visualization Parameters
  @Getter private double width = Units.inchesToMeters(24);

  @Getter
  private double xOffset = Units.inchesToMeters(0.); // x distance from robot center in meters

  @Getter
  private double yOffset = Units.inchesToMeters(0.); // y distance from robot center in meters

  @Getter
  private double zOffset = Units.inchesToMeters(0.); // z distance from robot center in meters

  @Getter private double slantAngle = 90.; // degrees from horizontal

  /**
   * @param masterMotor Device ID of the master motor in the gearbox
   */
  public ElevatorDescription addMasterMotor(CanDeviceId masterMotor) {
    this.masterMotor = masterMotor;
    return this;
  }
  /**
   * @param slaveMotor Device ID of the slave motor that will follow the master motor in the gearbox
   * @return
   */
  public ElevatorDescription addSlaveMotor(CanDeviceId slaveMotor) {
    this.slaveMotor = slaveMotor;
    return this;
  }
  /**
   * @param followInverted If set to true, the slave will output the inverse of the the master. If
   *     set to true, it will not be inverted.
   * @return
   */
  public ElevatorDescription withFollowInverted(boolean followInverted) {
    this.followInverted = followInverted;
    return this;
  }
  /**
   * @param gearing Gearing
   * @return
   */
  public ElevatorDescription withGearingToDrum(double gearing) {
    this.gearing = gearing;
    return this;
  }
  /**
   * @param radius Radius?
   * @return
   */
  public ElevatorDescription withDrumRadius(double radius) {
    this.drumRadiusInM = radius;
    return this;
  }
  /**
   * @param minHeight The minimum height the elevator can reach. Default value is zero. In meters.
   */
  public ElevatorDescription withMinHeight(double minHeight) {
    this.minHeight = minHeight;
    return this;
  }
  /**
   * @param maxHeight Maximum height of the elelevator in meters.
   * @return
   */
  public ElevatorDescription withMaxHeight(double maxHeight) {
    this.maxHeight = maxHeight;
    return this;
  }
  /**
   * @param height The starting height of the elevator in meters.
   * @return
   */
  public ElevatorDescription withStartingHeight(double height) {
    this.startingHeight = height;
    return this;
  }
  /**
   * @param mass The total mass of the elevator in kilograms.
   * @return
   */
  public ElevatorDescription withMassInKg(double mass) {
    this.massKg = mass;
    return this;
  }
  /**
   * @param config The pid config to be used in the physical hardware implementation.
   * @return
   */
  public ElevatorDescription withPIDGains(Slot0Configs config) {
    this.pidGains = config;
    return this;
  }
  /**
   * @param config PID gains to be used in simulation.
   * @return
   */
  public ElevatorDescription withSimulationPIDGains(Slot0Configs config) {
    this.simPidGains = config;
    return this;
  }
  /**
   * @param config MotionMagic configs to be used in the physcial hardware implmentation.
   * @return
   */
  public ElevatorDescription withMotionMagicConfigs(MotionMagicConfigs config) {
    this.motionMagicConfigs = config;
    return this;
  }
  /**
   * @param width Width of the elevator cube for visualization.
   * @return
   */
  public ElevatorDescription withWidth(double width) {
    this.width = width;
    return this;
  }
  /**
   * @param xInMeters X offset relative to drivebase.
   * @return
   */
  public ElevatorDescription withOffsetX(double xInMeters) {
    this.xOffset = xInMeters;
    return this;
  }
  /**
   * @param yInMeters Y offset relative to drivebase.
   * @return
   */
  public ElevatorDescription withOffsetY(double yInMeters) {
    this.yOffset = yInMeters;
    return this;
  }
  /**
   * @param zInMeters Z offset relative to drivebase.
   * @return
   */
  public ElevatorDescription withOffsetZ(double zInMeters) {
    this.zOffset = zInMeters;
    return this;
  }
  /**
   * @param angleInDegrees Angle of slant of elevator in degrees. 90 is straight up and down. 0 is
   *     horizontal.
   * @return
   */
  public ElevatorDescription withSlantAngle(double angleInDegrees) {
    this.slantAngle = angleInDegrees;
    return this;
  }

  public ElevatorDescription withLowerLimitSwitch(int port) {
    this.limitSwitchPort = port;
    return this;
  }

  /**
   * @param outputToHeight The conversion factor from the output shaft of the gearbox to the height
   *     of the elevator.
   * @return
   */
  public ElevatorDescription withOutputToHeight(double outputToHeight) {
    this.outputToHeight = outputToHeight;
    return this;
  }
}
