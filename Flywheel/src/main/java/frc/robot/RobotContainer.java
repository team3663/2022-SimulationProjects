package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.MoveElevatorCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;

public class RobotContainer {

  /*
  public final FlywheelSubsystem flywheel = new FlywheelSubsystem(Constants.MOTOR_ID);
  private final Joystick controller = new Joystick(Constants.CONTROLLER_ID);

  public RobotContainer() {
    new POVButton(controller, 90).whenPressed(new InstantCommand(() -> flywheel.increaseSpeed(), flywheel));
    new POVButton(controller, 270).whenPressed(new InstantCommand(() -> flywheel.decreaseSpeed(), flywheel));

    // for simulation, set DS--keyboard0 setting--pov buttons
  }
  */

  private final Joystick controller = new Joystick(Constants.CONTROLLER_ID);
    private final ElevatorSubsystem elevator = new ElevatorSubsystem(Constants.MOTOR_ID);

    public RobotContainer() {
        //new POVButton(controller, 90).whenPressed(new InstantCommand(() -> elevator.increaseHeight(), elevator));
        //new POVButton(controller, 270).whenPressed(new InstantCommand(() -> elevator.decreaseHeight(), elevator));
    }

    public Command getAutonomousCommand() {
      return new SequentialCommandGroup(new MoveElevatorCommand(elevator, 0.5),
                                        new WaitCommand(2),
                                        new MoveElevatorCommand(elevator, 1),
                                        new WaitCommand(1),
                                        new MoveElevatorCommand(elevator, 0));
    }

    // elevator 0 ->0.5, wait 2s at 0.5, ->1, wait 1s, ->0
}
