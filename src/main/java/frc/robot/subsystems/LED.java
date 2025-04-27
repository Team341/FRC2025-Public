// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.SuperStructure.Superstructure;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.coral.Coral;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.subsystems.wrist.Wrist;

public class LED extends SubsystemBase {

  private static LED instance;

  private final int LED_PORT = 1;

  private final int AMP_LED_START_INDEX = 8;
  private final int CANDLE_START_INDEX = 0;
  private final int CANDLE_LED_LENGTH = 7;

  private final int AMP_LED_END_INDEX = 109;

  public static boolean aligning;
  public static boolean isAligned;
  public static boolean blinking;
  public static boolean climbed = false;

  public static LED getInstance() {
    if (instance == null) {
      return instance = new LED();
    } else {
      return instance;
    }
  }

  private CANdle mLED;
  private boolean isAuto;

  private Animation m_toAnimate = null;

  private CANdleConfiguration config;

  /** Creates a new LED. */
  public LED() {
    isAuto = false;
    mLED = new CANdle(LED_PORT);
    config = new CANdleConfiguration();
    config.statusLedOffWhenActive = false;
    config.disableWhenLOS = false;
    config.stripType = LEDStripType.RGB;
    config.brightnessScalar = 0.5;
    config.vBatOutputMode = VBatOutputMode.On;

    mLED.configAllSettings(config, 100);
    // mLED.configFactoryDefault();

    m_toAnimate = new SingleFadeAnimation(50, 2, 200, 0, 0.5, AMP_LED_END_INDEX);
  }

  public void setAutoLED() {
    isAuto = true;
  }

  public void setTeleOpLED() {
    isAuto = false;
  }

  public boolean isUsingAutoLED() {
    return isAuto;
  }

  public void setAmpLEDs(Color color) {
    mLED.clearAnimation(0);
    mLED.setLEDs(
        (int) (color.red * 255),
        (int) (color.green * 255),
        (int) (color.blue * 255),
        255,
        AMP_LED_START_INDEX,
        AMP_LED_END_INDEX);
  }

  public void setLeftAmpLEDs(Color color) {
    mLED.clearAnimation(0);
    mLED.setLEDs(
        (int) (color.red * 255),
        (int) (color.green * 255),
        (int) (color.blue * 255),
        255,
        AMP_LED_START_INDEX,
        (AMP_LED_END_INDEX - AMP_LED_START_INDEX + 1) / 2);
  }

  public void setRightAmpLEDs(Color color) {
    mLED.clearAnimation(0);
    mLED.setLEDs(
        (int) (color.red * 255),
        (int) (color.green * 255),
        (int) (color.blue * 255),
        255,
        (AMP_LED_END_INDEX - AMP_LED_START_INDEX + 1) / 2 + 1,
        (AMP_LED_END_INDEX - AMP_LED_START_INDEX + 1) / 2);
  }

  public void setCANDLELED(Color color) {
    mLED.setLEDs(
        (int) (color.red * 255),
        (int) (color.green * 255),
        (int) (color.blue * 255),
        255,
        CANDLE_START_INDEX,
        CANDLE_LED_LENGTH);
  }

  public void setCANDLEAnimation(Color color) {
    mLED.animate(
        new StrobeAnimation(
            (int) (color.red * 255),
            (int) (color.green * 255),
            (int) (color.blue * 255),
            255,
            1,
            CANDLE_LED_LENGTH));
  }

  public void setAMPAnimation(Color color) {
    mLED.animate(
        new SingleFadeAnimation(
            (int) (color.red * 255),
            (int) (color.green * 255),
            (int) (color.blue * 255),
            255,
            0.3,
            AMP_LED_END_INDEX - AMP_LED_START_INDEX,
            AMP_LED_START_INDEX + 1));
  }

  public void setAMPRainbow() {
    mLED.animate(
        new RainbowAnimation(
            255, 0.3, AMP_LED_END_INDEX - AMP_LED_START_INDEX, false, AMP_LED_START_INDEX + 1));
  }

  public void setCANDLELEDIndex(Color color, int idx) {
    mLED.setLEDs(
        (int) (color.red * 255),
        (int) (color.green * 255),
        (int) (color.blue * 255),
        255,
        CANDLE_START_INDEX + idx,
        1);
  }

  /**
   * @return bus voltage
   */
  public double getBatteryVoltage() {
    return mLED.getBusVoltage();
  }

  /**
   * @return rail voltage
   */
  public double get5V() {
    return mLED.get5VRailVoltage();
  }

  /**
   * @return low side current
   */
  public double getCurrent() {
    return mLED.getCurrent();
  }

  /**
   * @return temperature in celcius
   */
  public double getTemperature() {
    return mLED.getTemperature();
  }

  /**
   * @param percent scaling brightness
   */
  public void configBrightness(double percent) {
    mLED.configBrightnessScalar(percent, 0);
  }

  /**
   * @param disableWhenLos whether to disable LED when signal is lost
   */
  public void configLos(boolean disableWhenLos) {
    mLED.configLOSBehavior(disableWhenLos, 0);
  }

  /**
   * @param type the type of LED
   */
  public void configLedType(LEDStripType type) {
    mLED.configLEDType(type, 0);
  }

  /**
   * @param offWhenActive whether LED is off when CANdle is activated
   */
  public void configStatusLedBehavior(boolean offWhenActive) {
    mLED.configStatusLedState(offWhenActive, 0);
  }

  public static double last_time = Timer.getFPGATimestamp();
  public boolean previousAlgaeInput = false;
  public boolean previousHasPiece = false;
  boolean intakeBlink = false;
  int intakeBlinkCount = 0;

  public void updateAnimation(
      Boolean isAlgae,
      Boolean isFlipped,
      Boolean elevatorLimit,
      Boolean coralLimit,
      Boolean gripperBreak,
      Boolean wristBreak,
      Boolean climberZero,
      Boolean climberAcquisition,
      Boolean hasPiece,
      Boolean passivePulse,
      Boolean gripperOverheat) {

    if (elevatorLimit) {
      setCANDLELEDIndex(Color.kGreen, 2);
    } else {
      setCANDLELEDIndex(Color.kRed, 2);
    }
    if (coralLimit) {
      setCANDLELEDIndex(Color.kGreen, 3);
    } else {
      setCANDLELEDIndex(Color.kRed, 3);
    }

    if (gripperBreak) {
      setCANDLELEDIndex(Color.kGreen, 7);
      ;
    } else {
      setCANDLELEDIndex(Color.kRed, 7);
    }

    if (wristBreak) {
      setCANDLELEDIndex(Color.kGreen, 6);
      ;
    } else {
      setCANDLELEDIndex(Color.kRed, 6);
    }

    if (climberZero) {
      setCANDLELEDIndex(Color.kGreen, 4);
      ;
    } else {
      setCANDLELEDIndex(Color.kRed, 4);
    }

    if (climberAcquisition) {
      setCANDLELEDIndex(Color.kGreen, 5);
      ;
    } else {
      setCANDLELEDIndex(Color.kRed, 5);
    }

    // subsequent is for led's on side elevator

    if (hasPiece != previousHasPiece && hasPiece == true) {
      previousHasPiece = hasPiece;

      intakeBlink = true;
    } else if (hasPiece == false) {
      previousHasPiece = false;
    }
    if (passivePulse) {
      setAMPAnimation(Color.kYellow);

    } else if (isAlgae && gripperOverheat) {
      clearAnimation();
      if (Timer.getFPGATimestamp() - last_time >= 0.4) {
        setAmpLEDs(Color.kDarkRed);
        if (Timer.getFPGATimestamp() - last_time >= 0.8) {
          last_time = Timer.getFPGATimestamp();
        }

      } else {
        setAmpLEDs(Color.kDarkBlue);
      }

    } else if (intakeBlink) {
      clearAnimation();

      if (Timer.getFPGATimestamp() - last_time >= 0.1) {
        setAmpLEDs(Color.kAntiqueWhite);
        if (Timer.getFPGATimestamp() - last_time >= 0.2) {
          last_time = Timer.getFPGATimestamp();
        }

      } else {
        setAmpLEDs(Color.kDarkGreen);
      }
      intakeBlinkCount++;

      if (intakeBlinkCount > 50) {
        previousHasPiece = hasPiece;
        intakeBlink = false;
        intakeBlinkCount = 0;
      }

    } else if (blinking) {
      clearAnimation();

      if (Timer.getFPGATimestamp() - last_time >= 0.1) {
        setAmpLEDs(Color.kAntiqueWhite);
        if (Timer.getFPGATimestamp() - last_time >= 0.2) {
          last_time = Timer.getFPGATimestamp();
        }

      } else {
        setAmpLEDs(Color.kDarkOrange);
      }

    } else if (aligning) {
      clearAnimation();
      if (isAligned) {
        setAmpLEDs(Color.kGreen);
      } else {
        setAmpLEDs(Color.kRed);
      }
    } else if (isAligned) {
      clearAnimation();
      if (isAligned) {
        setAmpLEDs(Color.kGreen);
      } else {
        setAmpLEDs(Color.kRed);
      }
    } else if (climbed) {
      setAMPRainbow();

    } else if (isAlgae) {
      if (previousAlgaeInput != isAlgae) {
        clear();
      }
      setAmpLEDs(Color.kBlue);

    } else if (isFlipped) {
      setAmpLEDs(Color.kCoral);

    } else if (!isAlgae) {
      if (previousAlgaeInput != isAlgae) {
        clear();
      }
      setAmpLEDs(new Color(125, 125, 125));
    }

    previousAlgaeInput = isAlgae;
  }

  /**
   * @return LED
   */
  public CANdle getLED() {
    return mLED;
  }

  @Override
  public void periodic() {
    updateAnimation(
        Superstructure.getInstance().isAlgae(),
        Superstructure.isFlipped,
        false,
        Coral.getInstance().getLimit(),
        Gripper.getInstance(Shoulder.getInstance().getAngleSupplier()).hasPiece().getAsBoolean(),
        Wrist.getInstance().getBeamBreak(),
        Climber.getInstance().getLimit(),
        false,
        Gripper.getInstance(Shoulder.getInstance().getAngleSupplier()).hasPiece().getAsBoolean(),
        Robot.passivePulsing,
        Gripper.getInstance(Shoulder.getInstance().getAngleSupplier()).getTemp() > 70);
  }

  public void clear() {
    getLED().clearAnimation(0);
    setAmpLEDs(Color.kBlack);
  }

  public void clearAnimation() {
    getLED().clearAnimation(0);
    mLED.clearAnimation(0);
  }
}
