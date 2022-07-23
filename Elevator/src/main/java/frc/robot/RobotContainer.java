package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.MoveElevatorCommand;
import frc.robot.commands.MoveElevatorDownCommand;
import frc.robot.commands.MoveElevatorUpCommand;
import frc.robot.subsystems.ElevatorSubsystem;

import static frc.robot.Constants.*;

public class RobotContainer {

    private final Joystick controller = new Joystick(CONTROLLER_ID);
    private final ElevatorSubsystem elevator = new ElevatorSubsystem(MOTOR_ID);

    public RobotContainer() {
        new POVButton(controller, 90).whenPressed(new MoveElevatorUpCommand(elevator, 0.2));
        new POVButton(controller, 270).whenPressed(new MoveElevatorDownCommand(elevator, 0.2));
    }

    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(new MoveElevatorCommand(elevator, 0.5),
                                          new WaitCommand(2),
                                          new MoveElevatorCommand(elevator, 1),
                                          new WaitCommand(1),
                                          new MoveElevatorCommand(elevator, 0));
      }
}