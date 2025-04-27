// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static final RobotIdentity identity = RobotIdentity.MISS_DAISY;
  public static final String CANIVORE_NAME = "CANIVORE 3";
  public static final Field currentField = Field.DCMP;
  CurrentLimitsConfigs talonCurrentLimitConfig;

  public static enum Field {
    CYBERSONICS,
    N03,
    HH,
    HHPRACTICE,
    SCH,
    SCHPRACTICE,
    DCMP,
    DCMPPRACTICE,
    WC,
    WCPRACTICE,
    ENERGY,
    MASS,
    NORMAL,
    KRYPTON
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public enum RobotIdentity {
    MISS_DAISY,
    KITBOT,
    BETABOT,
    SIMULATION
  }

  public Constants() {
    talonCurrentLimitConfig = new CurrentLimitsConfigs();
    talonCurrentLimitConfig.StatorCurrentLimit = 80;
    talonCurrentLimitConfig.SupplyCurrentLimit = 105;
    talonCurrentLimitConfig.StatorCurrentLimitEnable = true;
    talonCurrentLimitConfig.SupplyCurrentLimitEnable = true;
  }
}
