package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class MoveElevatorUpCommand extends CommandBase {
  
  private ElevatorSubsystem elevator;
  private double targetHeightIncrement;

  public MoveElevatorUpCommand(ElevatorSubsystem elevator, double targetHeightIncrement) {
    this.elevator = elevator;
    this.targetHeightIncrement = targetHeightIncrement;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    double targetHeight = elevator.getTargetHeight() + targetHeightIncrement;
    elevator.setTargetHeight(targetHeight);
  }

  @Override
  public boolean isFinished() {
    return elevator.atTargetHeight();
  }
}
