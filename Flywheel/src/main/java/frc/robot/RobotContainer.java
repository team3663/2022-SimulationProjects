package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.FlywheelSubsystem;

public class RobotContainer {

  public final FlywheelSubsystem flywheel = new FlywheelSubsystem(Constants.MOTOR_ID);
  private final Joystick controller = new Joystick(Constants.CONTROLLER_ID);

  public RobotContainer() {
    new POVButton(controller, 90).whenPressed(new InstantCommand(() -> flywheel.increaseSpeed(), flywheel));
    new POVButton(controller, 270).whenPressed(new InstantCommand(() -> flywheel.decreaseSpeed(), flywheel));

    // for simulation, set DS--keyboard0 setting--pov buttons
  }
}
