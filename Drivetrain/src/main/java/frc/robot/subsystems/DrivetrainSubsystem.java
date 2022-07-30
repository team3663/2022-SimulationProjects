// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N7;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class DrivetrainSubsystem extends SubsystemBase {

  private final WPI_TalonFX leftMotor;
  private final WPI_TalonFX rightMotor;
  private final AnalogGyro gyro;

  private final DifferentialDriveOdometry odometry;
  private final Field2d field;

  private final AnalogGyroSim gyroSim;
  private final DifferentialDrivetrainSim driveSim;

  private static final double GEARING = 7.29;
  private static final double MASS = 60.;
  private static final double TRACKWIDTH = 1.5;
  private static final double WHEEL_RADIUS = Units.inchesToMeters(3);
  private static final double MOMENT_OF_INERTIA = 7.5;
  private static final Matrix<N7, N1> SD = VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005);

  private static final double kP = 0.5;
  private static final int PRIMARY_PID = 0;
  private static final int SLOT_INDEX = 0;
  private static final int TIMEOUT = 20;

  private static final double SENSOR_POSITION_FACTOR = 2048.0 * GEARING / (2 * Math.PI);
  private static final double SENSOR_VELOCITY_FACTOR = SENSOR_POSITION_FACTOR / 10.0;

  private double leftSpeed = 0;
  private double rightSpeed = 0;

  public DrivetrainSubsystem(int leftMotorID, int rightMotorID, int analogChannel) {
    leftMotor = new WPI_TalonFX(leftMotorID);
    rightMotor = new WPI_TalonFX(rightMotorID);
    gyro = new AnalogGyro(analogChannel);

    leftMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, PRIMARY_PID, TIMEOUT);
    leftMotor.config_kP(SLOT_INDEX, kP);
    rightMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, PRIMARY_PID, TIMEOUT);
    rightMotor.config_kP(SLOT_INDEX, kP);

    odometry = new DifferentialDriveOdometry(new Rotation2d());
    field = new Field2d();

    driveSim = new DifferentialDrivetrainSim(DCMotor.getFalcon500(2), GEARING, MOMENT_OF_INERTIA, MASS, WHEEL_RADIUS, TRACKWIDTH, SD);
    gyroSim = new AnalogGyroSim(gyro);

    ShuffleboardTab tab = Shuffleboard.getTab("drivetrain");
    tab.add("field", field);
    tab.addNumber("left pose", () -> getLeftPosition());
    tab.addNumber("right pose", () -> getRightPosition());
  }

  public void drive(double leftSpeed, double rightSpeed) {
    this.leftSpeed = leftSpeed;
    this.rightSpeed = rightSpeed;
  }

  public double getLeftPosition() {
    return leftMotor.getSelectedSensorPosition() / SENSOR_POSITION_FACTOR;
  }

  public double getRightPosition() {
    return rightMotor.getSelectedSensorPosition() / SENSOR_POSITION_FACTOR;
  }

  @Override
  public void periodic() {
    leftMotor.set(ControlMode.PercentOutput, leftSpeed);
    rightMotor.set(ControlMode.PercentOutput, rightSpeed);

    odometry.update(gyro.getRotation2d(), getLeftPosition(), getRightPosition());
    field.setRobotPose(odometry.getPoseMeters());
  }

  @Override
  public void simulationPeriodic() {
    TalonFXSimCollection leftSimCollection = leftMotor.getSimCollection();
    TalonFXSimCollection rightSimCollection = rightMotor.getSimCollection();

    driveSim.setInputs(leftSimCollection.getMotorOutputLeadVoltage(), rightSimCollection.getMotorOutputLeadVoltage());
    driveSim.update(Robot.kDefaultPeriod);

    double leftPosition = driveSim.getLeftPositionMeters() * SENSOR_POSITION_FACTOR;
    double rightPosition = driveSim.getRightPositionMeters() * SENSOR_POSITION_FACTOR;
    double leftVelocity = driveSim.getLeftVelocityMetersPerSecond() * SENSOR_VELOCITY_FACTOR;
    double rightVelocity = driveSim.getRightVelocityMetersPerSecond() * SENSOR_VELOCITY_FACTOR;

    leftSimCollection.setIntegratedSensorRawPosition((int)leftPosition);
    rightSimCollection.setIntegratedSensorRawPosition((int)rightPosition);
    leftSimCollection.setIntegratedSensorVelocity((int)leftVelocity);
    rightSimCollection.setIntegratedSensorVelocity((int)rightVelocity);

    gyroSim.setAngle(driveSim.getHeading().getDegrees());
  }
}
