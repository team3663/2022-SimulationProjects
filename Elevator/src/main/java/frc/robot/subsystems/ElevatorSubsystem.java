package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

  private WPI_TalonFX motor;
  private ElevatorSim sim;

  private double targetHeight = 0;

  private double kP = 0.2;
  private double kD = 0.1;

  private static final double GEARING = 10;
  private static final double CARRIAGE_MASS = 0.5; // kg
  private static final double DRUM_RADIUS = 0.2; // m
  private static final double MIN_HEIGHT = 0;
  private static final double MAX_HEIGHT = 1;

  private static final double SENSOR_POSITION_FACTOR = 2048 * GEARING / 2 / Math.PI / DRUM_RADIUS; // ticks/m
  // converted from 2048 ticks / motor rotation

  private double heightIncrement = 0.2;

  public ElevatorSubsystem(int motorID) {
    motor = new WPI_TalonFX(motorID);
    
    motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 250);
    motor.config_kP(0, kP);
    motor.config_kD(0, kD);

    sim = new ElevatorSim(DCMotor.getFalcon500(2), GEARING, CARRIAGE_MASS, DRUM_RADIUS, MIN_HEIGHT, MAX_HEIGHT);

    ShuffleboardTab tab = Shuffleboard.getTab("elevator");
    tab.addNumber("current height", () -> getCurrentHeight());
    tab.addNumber("target height", () -> getTargetHeight());
    tab.addNumber("voltage", () -> motor.getMotorOutputVoltage());
  }

  public void increaseHeight() {
    targetHeight += heightIncrement;
  }

  public void decreaseHeight() {
    targetHeight -= heightIncrement;
  }

  public double getTargetHeight() {
    return targetHeight;
  }

  public double getCurrentHeight() {
    return motor.getSelectedSensorPosition() / SENSOR_POSITION_FACTOR;
  }

  @Override
  public void simulationPeriodic() {
    TalonFXSimCollection simCollection = motor.getSimCollection();

    sim.setInputVoltage(simCollection.getMotorOutputLeadVoltage());
    sim.update(0.02);

    simCollection.setIntegratedSensorRawPosition((int)(sim.getPositionMeters() * SENSOR_POSITION_FACTOR));
  }

  @Override
  public void periodic() {
    motor.set(TalonFXControlMode.Position, targetHeight * SENSOR_POSITION_FACTOR);
  }
}
