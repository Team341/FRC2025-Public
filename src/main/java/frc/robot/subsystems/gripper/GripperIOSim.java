package frc.robot.subsystems.gripper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import java.util.function.DoubleSupplier;

public class GripperIOSim implements GripperIO {
  private static boolean leftLimitHit = false;
  private static boolean rightLimitHit = false;
  private boolean isFlingMode = false;
  private boolean flinging = false;
  private DoubleSupplier shoulderAngle;
  private FlywheelSim gripperSim;
  private Notifier flingNotifier = new Notifier(() -> flingPeriodic());

  public GripperIOSim(DoubleSupplier shoulderAngle) {
    gripperSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 0.0006, 1 / 3.75),
            DCMotor.getKrakenX60(1));
    this.shoulderAngle = shoulderAngle;
  }

  private void flingPeriodic() {
    if (isFlingMode && MathUtil.isNear(-10, shoulderAngle.getAsDouble(), 2.5)) {
      runVolts(12);
      flinging = true;
    }
    if (flinging) runVolts(12);
  }

  @Override
  public void updateInputs(GripperIOInputs inputs) {

    gripperSim.update(.02);
    inputs.appliedVolts = gripperSim.getInputVoltage();
    inputs.supplyCurrent = gripperSim.getCurrentDrawAmps();
    inputs.torqueCurrent = 0;
    inputs.temperatureCelsius = 0.0;
    inputs.velocityRPM = gripperSim.getOutput(0);
    inputs.flingMode = isFlingMode;
    inputs.beamBreak = false;

    if (!isFlingMode) {
      flingNotifier.stop();
      if (flinging) {
        flinging = false;
        runVolts(0);
      }
      flinging = false;
    }
  }

  @Override
  public void runVolts(double volts) {
    gripperSim.setInputVoltage(volts);
  }

  @Override
  public void stop() {
    gripperSim.setInputVoltage(0);
  }

  @Override
  public void setFling(boolean fling) {
    isFlingMode = fling;
    if (fling) flingNotifier.startPeriodic(0.005);
  }

  public static void setLeftLimit(boolean hit) {
    leftLimitHit = hit;
  }

  public static void setRightLimit(boolean hit) {
    rightLimitHit = hit;
  }
}
