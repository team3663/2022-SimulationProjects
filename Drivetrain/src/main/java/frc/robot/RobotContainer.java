// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RobotContainer {
  private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem(Constants.LEFT_MOTOR_ID, Constants.RIGHT_MOTOR_ID, Constants.GYRO_CHANNEL);
  private final XboxController controller = new XboxController(Constants.CONTROLLER_ID);

  public RobotContainer() {
    drivetrain.setDefaultCommand(new DriveCommand(drivetrain, () -> controller.getLeftY(), () -> controller.getRightY()));
  }
}
