package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.MoveArmDownCommand;
import frc.robot.commands.MoveArmUpCommand;
import frc.robot.subsystems.ArmSubsystem;

public class RobotContainer {
  private final ArmSubsystem arm = new ArmSubsystem(Constants.MOTOR_ID);
  private final Joystick controller = new Joystick(Constants.CONTROLLER_ID);

  public RobotContainer() {
      new POVButton(controller, 90).whenPressed(new MoveArmUpCommand(arm, 10));
      new POVButton(controller, 270).whenPressed(new MoveArmDownCommand(arm, 10));
  }

}
