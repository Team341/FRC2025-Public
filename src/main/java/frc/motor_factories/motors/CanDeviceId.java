// Copyright (c) 2023 FRC 254
// https://github.com/Team254/FRC-2023-Public
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.motor_factories.motors;

import com.ctre.phoenix6.signals.InvertedValue;

public class CanDeviceId {
  private final int deviceNumber;
  private final String bus;
  private final InvertedValue direction;

  public CanDeviceId(int deviceNumber, String bus, InvertedValue direction) {
    this.deviceNumber = deviceNumber;
    this.bus = bus;
    this.direction = direction;
  }

  // use the default motor direction "Clockwise_Positive"
  public CanDeviceId(int deviceNumber, String bus) {
    this(deviceNumber, bus, InvertedValue.Clockwise_Positive);
  }

  // Use the default bus name "rio".
  public CanDeviceId(int deviceNumber) {
    this(deviceNumber, "rio", InvertedValue.Clockwise_Positive);
  }

  public int getDeviceNumber() {
    return deviceNumber;
  }

  public String getBus() {
    return bus;
  }

  public InvertedValue getMotorDirection() {
    return direction;
  }

  @SuppressWarnings("NonOverridingEquals")
  public boolean equals(CanDeviceId other) {
    return other.deviceNumber == deviceNumber && other.bus.equals(bus);
  }

  @Override
  public String toString() {
    return "CanDeviceId(" + deviceNumber + ", " + bus + ", " + direction + ")";
  }
}
