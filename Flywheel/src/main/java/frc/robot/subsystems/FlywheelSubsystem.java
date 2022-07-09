package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlywheelSubsystem extends SubsystemBase {

  private final WPI_TalonFX motor;
  private final FlywheelSim sim;

  private int targetSpeed = 0;
  private int speedIncrement = 100;

  private static final double SENSOR_POSITION_FACTOR = 2048 / 2 / Math.PI; //ticks per rad
  private static final double SENSOR_VELOCITY_FACTOR = SENSOR_POSITION_FACTOR / 10 ; // 100ms
  private static final double GEARING = 1; // motor input : gear box output
  private static final double MOMENT_OF_INERTIA = 0.1;

  public static final int MAX_SPEED = 3500; // sim doesn't work for 4000+

  public static final double FF_VELOCITY_CONSTANT = 12 / Units.rotationsPerMinuteToRadiansPerSecond(6380);
  // feedforward: u = kV * v + kA * a (in this case we don't need a)
  // targetVoltage(u) / maxVoltage = targetVelocity / maxVelocity
  // u = maxVoltage / max velocity * targetVelocity

  // another note on voltage
  // v = IR + w/kW
  // IR accounts for acclearation, w/kW accounts for rotational velocity

  private double kP = 1;
  // ideal value = 12 / Units.rotationsPerMinuteToRadiansPerSecond(250);

  private double ffVoltage = 0;
  private double pidVoltage = 0;
  // u = ffVoltage + pidVoltage
  // ff brings it somewhere near the target (ffVoltage is constant)
  // fb(pid) gets it accurate

  public FlywheelSubsystem(int motorID) {
    motor = new WPI_TalonFX(motorID);
    motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 250);
    motor.config_kP(0, kP);

    sim = new FlywheelSim(DCMotor.getFalcon500(2), GEARING, MOMENT_OF_INERTIA);

    ShuffleboardTab tab = Shuffleboard.getTab("shooter");
    tab.addNumber("current velocity", () -> Units.radiansPerSecondToRotationsPerMinute(getCurrentSpeed()));
    tab.addNumber("target velocity", () -> Units.radiansPerSecondToRotationsPerMinute(getTargetSpeed()));
    tab.addNumber("voltage", () -> motor.getSimCollection().getMotorOutputLeadVoltage());
    tab.addNumber("ffVoltage", () -> ffVoltage);
    tab.addNumber("pidVoltage", () -> pidVoltage);
  }

  public void increaseSpeed() {
      targetSpeed += speedIncrement;
      setSpeed(targetSpeed);
  }

  public void decreaseSpeed() {
      targetSpeed -= speedIncrement;
      setSpeed(targetSpeed);
  }

  public void setSpeed(int targetSpeed) {
    if (targetSpeed > MAX_SPEED) {
      targetSpeed = MAX_SPEED;
    }
    else if (targetSpeed < 0) {
      targetSpeed = 0;
    }

    this.targetSpeed = targetSpeed;
  }

  public double getCurrentSpeed() {
    return motor.getSelectedSensorVelocity() / SENSOR_VELOCITY_FACTOR; // sensor to real units
  }

  public double getTargetSpeed() {
    return targetSpeed;
  }

  @Override
  public void simulationPeriodic() {
    TalonFXSimCollection simCollection = motor.getSimCollection();

    sim.setInputVoltage(simCollection.getMotorOutputLeadVoltage());
    sim.update(0.02);

    simCollection.setIntegratedSensorVelocity((int)(sim.getAngularVelocityRadPerSec() * SENSOR_VELOCITY_FACTOR)); // real to sensor units
  }

  @Override
  public void periodic() {
    ffVoltage = FF_VELOCITY_CONSTANT * targetSpeed;
    motor.set(TalonFXControlMode.Velocity, targetSpeed * SENSOR_VELOCITY_FACTOR, DemandType.ArbitraryFeedForward, ffVoltage / 12);
  }
}