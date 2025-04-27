// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SuperStructure;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import lombok.Getter;

/** Add your docs here. */
public class RobotPose {
  private double wristAngle;
  private double shoulderAngle;
  private double elevatorHeight;
  private double coralIntakeAngle;
  private double coralIntakeSpeed = 0;
  private double gripperSpeed = 0;

  @Getter
  private MotionMagicConfigs elevatorConfigs =
      new MotionMagicConfigs().withMotionMagicCruiseVelocity(85).withMotionMagicAcceleration(185);

  private MotionMagicConfigs shoulderConfigs =
      new MotionMagicConfigs().withMotionMagicCruiseVelocity(2).withMotionMagicAcceleration(9.);

  public double getGripperSpeed() {
    return gripperSpeed;
  }

  public void setGripperSpeed(double gripperSpeed) {
    this.gripperSpeed = gripperSpeed;
  }

  public RobotPose setEleavtorConfigs(MotionMagicConfigs elevatorConfig) {
    this.elevatorConfigs = elevatorConfig;
    return this;
  }

  public MotionMagicConfigs getShoulderConfigs() {
    return shoulderConfigs;
  }

  public RobotPose setShoulderConfigs(MotionMagicConfigs shoulderConfigs) {
    this.shoulderConfigs = shoulderConfigs;
    return this;
  }

  public RobotPose(double elevatorHeight, double shoulderAngle, double wristAngle) {
    this.elevatorHeight = elevatorHeight;
    this.shoulderAngle = shoulderAngle;
    this.wristAngle = wristAngle;
    this.coralIntakeAngle = Math.PI / 2.;
    this.coralIntakeSpeed = 0;
  }

  public RobotPose(
      double elevatorHeight, double shoulderAngle, double wristAngle, double coralIntakeAngle) {
    this.elevatorHeight = elevatorHeight;
    this.shoulderAngle = shoulderAngle;
    this.wristAngle = wristAngle;
    this.coralIntakeAngle = coralIntakeAngle;
    this.coralIntakeSpeed = 0;
  }

  public RobotPose(
      double elevatorHeight,
      double shoulderAngle,
      double wristAngle,
      double coralIntakeAngle,
      double coralIntakeSpeed,
      double gripperSpeed) {

    this.elevatorHeight = elevatorHeight;
    this.shoulderAngle = shoulderAngle;
    this.wristAngle = wristAngle;
    this.coralIntakeAngle = coralIntakeAngle;
    this.coralIntakeSpeed = coralIntakeSpeed;
    this.gripperSpeed = gripperSpeed;
  }

  public double getCoralIntakeSpeed() {
    return coralIntakeSpeed;
  }

  public void setCoralIntakeSpeed(double coralIntakeSpeed) {
    this.coralIntakeSpeed = coralIntakeSpeed;
  }

  public double getWristAngle() {
    return this.wristAngle;
  }

  public double getCoralIntakeAngle() {
    return coralIntakeAngle;
  }

  public void setCoralIntakeAngle(double coralIntakeAngle) {
    this.coralIntakeAngle = coralIntakeAngle;
  }

  public double getShoulderAngle() {
    return shoulderAngle;
  }

  public double getElevatorHeight() {
    return elevatorHeight;
  }

  public void setWristAngle(double wristAngle) {
    this.wristAngle = wristAngle;
  }

  public void setShoulderAngle(double armAngle) {
    this.shoulderAngle = armAngle;
  }

  public void setElevatorHeight(double elevatorHeight) {
    this.elevatorHeight = elevatorHeight;
  }
}
