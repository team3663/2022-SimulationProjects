// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class ArmSubsystem extends SubsystemBase {

  private final SingleJointedArmSim sim;
  private final WPI_TalonFX motor;

  private static final double kP = 0.1;
  private static final double kI = 0;
  private static final double kD = 0;
  private static final double kF = 0;
  private static final int SLOT_INDEX = 0;
  private static final int PRIMARY_PID = 0;
  private static final int TIMEOUT = 20;
  private static final double ANGLE_TOLERANCE = Units.degreesToRadians(2);

  private static final double GEARING = 10;
  private static final double ARM_LENGTH = 0.5;
  private static final double MIN_ANGLE = 0;
  private static final double MAX_ANGLE = Math.PI * 2;
  private static final double MASS = 5;
  private static final double MOMENT_OF_INERTIA = MASS * ARM_LENGTH * ARM_LENGTH;

  private static final double SENSOR_POSITION_FACTOR = 2048.0 * GEARING / (2 * Math.PI);
  private static final double SENSOR_VELOCITY_FACTOR = SENSOR_POSITION_FACTOR / 10.0;

  private double targetAngle;

  public ArmSubsystem(int motorID) {
    motor = new WPI_TalonFX(motorID);
    
    motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, PRIMARY_PID, TIMEOUT);
    motor.config_kP(SLOT_INDEX, kP);
    motor.config_kI(SLOT_INDEX, kI);
    motor.config_kD(SLOT_INDEX, kD);
    motor.config_kF(SLOT_INDEX, kF);
    motor.selectProfileSlot(SLOT_INDEX, PRIMARY_PID);
    motor.setSelectedSensorPosition(0, PRIMARY_PID, TIMEOUT);

    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, TIMEOUT);

    motor.configNominalOutputForward(0);
    motor.configNominalOutputReverse(0);
    motor.configPeakOutputForward(1);
    motor.configPeakOutputReverse(-1);

    motor.configMotionCruiseVelocity(Units.degreesToRadians(10) * SENSOR_VELOCITY_FACTOR);
    motor.configMotionAcceleration(Units.degreesToRadians(20) * SENSOR_VELOCITY_FACTOR);

    sim = new SingleJointedArmSim(DCMotor.getFalcon500(2), GEARING, MOMENT_OF_INERTIA, ARM_LENGTH, MIN_ANGLE, MAX_ANGLE, MASS, false);

    ShuffleboardTab tab = Shuffleboard.getTab("arm");
    tab.addNumber("current angle", () -> Units.radiansToDegrees(getCurrentAngle()));
    tab.addNumber("target angle", () -> Units.radiansToDegrees(getTargetAngle()));
    tab.addNumber("voltage", () -> motor.getMotorOutputVoltage());
    tab.addNumber("error", () -> Units.radiansToDegrees(motor.getClosedLoopError() / SENSOR_POSITION_FACTOR));
  }

  public void setTargetAngle(double targetAngle) {
    this.targetAngle = targetAngle;
  }

  public double getTargetAngle() {
    return targetAngle;
  }

  public double getCurrentAngle() {
    return motor.getSelectedSensorPosition() / SENSOR_POSITION_FACTOR;
  }

  public boolean atTargetAngle() {
    return Math.abs(getTargetAngle() - getCurrentAngle()) < ANGLE_TOLERANCE;
  }

  @Override
  public void simulationPeriodic() {
    TalonFXSimCollection simCollection = motor.getSimCollection();

    sim.setInputVoltage(simCollection.getMotorOutputLeadVoltage());
    sim.update(Robot.kDefaultPeriod);

    double sensorPosition = sim.getAngleRads() * SENSOR_POSITION_FACTOR;
    simCollection.setIntegratedSensorRawPosition((int)sensorPosition);

    double sensorVelocity = sim.getVelocityRadPerSec() * SENSOR_VELOCITY_FACTOR;
    simCollection.setIntegratedSensorVelocity((int)sensorVelocity);
  }

  @Override
  public void periodic() {
    motor.set(TalonFXControlMode.Position, getTargetAngle() * SENSOR_POSITION_FACTOR);
    // motor.set(TalonFXControlMode.PercentOutput, 0.1);
  }
}
