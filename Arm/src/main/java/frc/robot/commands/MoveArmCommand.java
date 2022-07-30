package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmCommand extends CommandBase {

  private ArmSubsystem arm;
  private double targetAngle;

  public MoveArmCommand(ArmSubsystem arm, double targetAngle) {
    this.arm = arm;
    this.targetAngle = Units.degreesToRadians(targetAngle);

    addRequirements(arm);
  }

  @Override
  public void initialize() {
    arm.setTargetAngle(targetAngle);
  }

  @Override
  public boolean isFinished() {
    return arm.atTargetAngle();
  }
}
