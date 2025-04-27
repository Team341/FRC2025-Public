package frc.robot.subsystems.gripper;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import frc.motor_factories.motors.TalonFXFactory;
import frc.robot.subsystems.SuperStructure.Superstructure;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class GripperIOTalonFX implements GripperIO {
  // Hardware
  private final TalonFX gripperMotor;
  private final DigitalInput beamBreak;
  private final DoubleSupplier shoulderAngle;

  // Constants
  private static final double FLING_VOLTAGE = 12.0;
  private static final double FLING_ANGLE_TOLERANCE = 7.5;
  private static final double FRONT_TARGET_ANGLE = -35;
  private static final double REAR_TARGET_ANGLE = 35;
  private static final double NOTIFIER_PERIOD = 1.0 / 500; // 500Hz

  // State
  private boolean isFlingMode = false;
  private boolean flinging = false;
  private boolean notifierCalled = false;

  // CAN Optimization
  private final VoltageOut flingVoltageOut =
      new VoltageOut(FLING_VOLTAGE).withUpdateFreqHz(0).withEnableFOC(true);

  // Cached values
  private double cachedShoulderAngle = 0;
  private double lastShoulderAngle = 0;
  private double lastAngleUpdateTime = 0;
  private double angleVelocity = 0;

  // Latency tracking
  private final MovingAverage latencyFilter = new MovingAverage(10);
  private double predictedLatency = 2.; // ms initial estimate

  private final Notifier flingNotifier = new Notifier(this::flingPeriodic);

  public GripperIOTalonFX(DoubleSupplier shoulderAngle) {
    this.shoulderAngle = shoulderAngle;
    gripperMotor = TalonFXFactory.createDefaultTalon(GripperConstants.getGripperID());
    beamBreak = new DigitalInput(GripperConstants.getBeamBreakID());

    // Configure motor
    CurrentLimitsConfigs config = new CurrentLimitsConfigs();
    config.StatorCurrentLimit = 100;
    config.SupplyCurrentLimit = 100;
    config.SupplyCurrentLimitEnable = true;
    config.StatorCurrentLimitEnable = true;
    gripperMotor.getConfigurator().apply(config);

    // Set notifier priority
    flingNotifier.setHALThreadPriority(true, 60);
  }

  private void flingPeriodic() {
    double loopStartTime = Timer.getFPGATimestamp();
    updateAngleCache();

    // Main fling logic
    if (isFlingMode && !flinging) {
      double targetAngle = Superstructure.isFront ? FRONT_TARGET_ANGLE : REAR_TARGET_ANGLE;
      if (MathUtil.isNear(targetAngle, cachedShoulderAngle, FLING_ANGLE_TOLERANCE)) {
        gripperMotor.setControl(flingVoltageOut);
        flinging = true;
        Logger.recordOutput("Gripper/FlingState", true);
      }
    }

    if (flinging) {
      runVolts(FLING_VOLTAGE);
    }

    // Diagnostics
    notifierCalled = true;
    logDiagnostics(loopStartTime);
  }

  private void updateAngleCache() {
    double now = Timer.getFPGATimestamp();
    double elapsedSec = now - lastAngleUpdateTime;

    // only update if we will have new data
    if (elapsedSec >= 1 / 500) {
      lastShoulderAngle = cachedShoulderAngle;
      cachedShoulderAngle = shoulderAngle.getAsDouble();
      lastAngleUpdateTime = now;

      // Potential Can Latency Compensation
      if (elapsedSec > 0) {
        angleVelocity = (cachedShoulderAngle - lastShoulderAngle) / elapsedSec;
      }
    }

    cachedShoulderAngle += angleVelocity * (predictedLatency / 1000.0);
  }

  private void logDiagnostics(double loopStartTime) {
    // Timing diagnostics
    double loopTimeMs = (Timer.getFPGATimestamp() - loopStartTime) * 1000;
    Logger.recordOutput("Gripper/LoopTimeMs", loopTimeMs);

    // CAN latency
    double actualLatencyMs = (Timer.getFPGATimestamp() - lastAngleUpdateTime) * 1000;
    latencyFilter.add(actualLatencyMs);
    predictedLatency = latencyFilter.getAverage();
    Logger.recordOutput("Gripper/CANLatencyMs", predictedLatency);

    Logger.recordOutput("Gripper/CachedAngle", cachedShoulderAngle);
    Logger.recordOutput("Gripper/AngleVelocityDegPerSec", angleVelocity);
  }

  @Override
  public void updateInputs(GripperIOInputs inputs) {
    inputs.beamBreak = !beamBreak.get();
    inputs.appliedVolts = gripperMotor.getMotorVoltage().getValueAsDouble();
    inputs.supplyCurrent = gripperMotor.getSupplyCurrent().getValueAsDouble();
    inputs.temperatureCelsius = gripperMotor.getDeviceTemp().getValueAsDouble();
    inputs.torqueCurrent = gripperMotor.getStatorCurrent().getValueAsDouble();
    inputs.notifierCalled = notifierCalled;
    inputs.velocityRPM = gripperMotor.getVelocity().getValueAsDouble() * 60;
    inputs.flingMode = isFlingMode;

    if (!isFlingMode && flinging) {
      flingNotifier.stop();
      flinging = false;
      runVolts(0);
      notifierCalled = false;
    }
  }

  @Override
  public void runVolts(double volts) {
    gripperMotor.setControl(new VoltageOut(volts).withEnableFOC(true));
  }

  @Override
  public void stop() {
    gripperMotor.stopMotor();
  }

  @Override
  public void setMode(NeutralModeValue mode) {
    MotorOutputConfigs configs = new MotorOutputConfigs();
    configs.NeutralMode = mode;
    gripperMotor.getConfigurator().apply(configs);
  }

  @Override
  public void setFling(boolean enable) {
    isFlingMode = enable;
    if (enable) {
      flingNotifier.startPeriodic(NOTIFIER_PERIOD);
    } else {
      flingNotifier.stop();
    }
  }

  private static class MovingAverage {
    private final double[] values;
    private int index = 0;
    private boolean filled = false;

    MovingAverage(int size) {
      values = new double[size];
    }

    void add(double value) {
      values[index] = value;
      index = (index + 1) % values.length;
      if (index == 0) filled = true;
    }

    double getAverage() {
      int count = filled ? values.length : index;
      if (count == 0) return 0;

      double sum = 0;
      for (int i = 0; i < count; i++) {
        sum += values[i];
      }
      return sum / count;
    }
  }
}
