package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class MoveElevatorDownCommand extends CommandBase {
  
  private ElevatorSubsystem elevator;
  private double targetHeightIncrement;

  public MoveElevatorDownCommand(ElevatorSubsystem elevator, double targetHeightIncrement) {
    this.elevator = elevator;
    this.targetHeightIncrement = targetHeightIncrement;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    double targetHeight = elevator.getTargetHeight() - targetHeightIncrement;
    elevator.setTargetHeight(targetHeight);
  }
}
