package frc.robot.commands;

import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveCommand extends CommandBase {
  private final DrivetrainSubsystem drivetrain;
  private final DoubleSupplier leftSpeed;
  private final DoubleSupplier rightSpeed;

  public DriveCommand(DrivetrainSubsystem drivetrain, DoubleSupplier leftSpeed, DoubleSupplier rightSpeed) {
    this.drivetrain = drivetrain;
    this.leftSpeed = leftSpeed;
    this.rightSpeed = rightSpeed;

    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    drivetrain.drive(leftSpeed.getAsDouble(), rightSpeed.getAsDouble());
  }
}
