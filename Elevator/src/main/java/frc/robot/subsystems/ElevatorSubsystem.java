package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class ElevatorSubsystem extends SubsystemBase {

  private WPI_TalonFX motor;
  private ElevatorSim sim;

  private double targetHeight = 0;

  private double kP = 0.25;
  private double kI = 0; 
  private double kD = 0;
  private double kF = 0;

  private int timeout = 20; // ms
  private int primaryPID = 0;
  private int slotIndex = 0;

  private static final double GEARING = (36.0/16.0) * (44.0/20.0) * (48.0/18.0) ;
  private static final double CARRIAGE_MASS = Units.lbsToKilograms(60.0); 
  private static final double DRUM_RADIUS = Units.inchesToMeters(0.9);
  private static final double MIN_HEIGHT = 0;
  private static final double MAX_HEIGHT = 1;

  private static final double SENSOR_POSITION_FACTOR = 2048.0 * GEARING / (2.0 * Math.PI * DRUM_RADIUS); // ticks/m
  private static final double SENSOR_VELOCITY_FACTOR = SENSOR_POSITION_FACTOR / 10.0;
  // converted from 2048 ticks / motor rotation

  private static final double HEIGHT_TOLERANCE = 0.02;

  public ElevatorSubsystem(int motorID) {
    motor = new WPI_TalonFX(motorID);
    
    motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, primaryPID, timeout);
    motor.config_kP(slotIndex, kP);
    motor.config_kI(slotIndex, kI);
    motor.config_kD(slotIndex, kD);
    motor.config_kF(slotIndex, kF);
    motor.selectProfileSlot(slotIndex, primaryPID);
    motor.setSelectedSensorPosition(0, primaryPID, timeout);

    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, timeout);

    motor.configNominalOutputForward(0);
    motor.configNominalOutputReverse(0);
    motor.configPeakOutputForward(1);
    motor.configPeakOutputReverse(-1);

    motor.configMotionCruiseVelocity(Units.feetToMeters(6) * SENSOR_VELOCITY_FACTOR);
    motor.configMotionAcceleration(Units.feetToMeters(12) * SENSOR_VELOCITY_FACTOR);

    sim = new ElevatorSim(DCMotor.getFalcon500(2), GEARING, CARRIAGE_MASS, DRUM_RADIUS, MIN_HEIGHT, MAX_HEIGHT);

    ShuffleboardTab tab = Shuffleboard.getTab("elevator");
    tab.addNumber("current height", () -> getCurrentHeight());
    tab.addNumber("target height", () -> getTargetHeight());
    tab.addNumber("voltage", () -> motor.getMotorOutputVoltage());
    tab.addNumber("error", () -> motor.getClosedLoopError() / SENSOR_POSITION_FACTOR);
    tab.addBoolean("atTargetHeight", () -> atTargetHeight());
  }

  public void setTargetHeight(double targetHeight) {
    this.targetHeight = targetHeight;
  }

  public double getTargetHeight() {
    return targetHeight;
  }

  public double getCurrentHeight() {
    return motor.getSelectedSensorPosition() / SENSOR_POSITION_FACTOR;
  }

  public boolean atTargetHeight() {
    return Math.abs(getTargetHeight() - getCurrentHeight()) < HEIGHT_TOLERANCE;
  }

  @Override
  public void simulationPeriodic() {
    TalonFXSimCollection simCollection = motor.getSimCollection();

    sim.setInputVoltage(simCollection.getMotorOutputLeadVoltage());
    sim.update(Robot.kDefaultPeriod);

    double sensorPosition = sim.getPositionMeters() * SENSOR_POSITION_FACTOR;
    simCollection.setIntegratedSensorRawPosition((int)sensorPosition);

    double sensorVelocity = sim.getVelocityMetersPerSecond() * SENSOR_VELOCITY_FACTOR;
    simCollection.setIntegratedSensorVelocity((int)sensorVelocity);
  }

  @Override
  public void periodic() {
    motor.set(TalonFXControlMode.MotionMagic, getTargetHeight() * SENSOR_POSITION_FACTOR);
  }
}
