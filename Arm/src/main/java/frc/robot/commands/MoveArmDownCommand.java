package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveArmDownCommand extends CommandBase {
  private final ArmSubsystem arm;
  private final double targetAngleIncrement;

  public MoveArmDownCommand(ArmSubsystem arm, double targetAngleIncrement) {
    this.arm = arm;
    this.targetAngleIncrement = targetAngleIncrement;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    double targetAngle = arm.getTargetAngle() - Units.degreesToRadians(targetAngleIncrement);
    arm.setTargetAngle(targetAngle);
  }

  @Override
  public boolean isFinished() {
    return arm.atTargetAngle();
  }
}
