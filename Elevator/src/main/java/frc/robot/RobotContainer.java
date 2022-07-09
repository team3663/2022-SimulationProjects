package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.ElevatorSubsystem;

import static frc.robot.Constants.*;

public class RobotContainer {

    private final Joystick controller = new Joystick(CONTROLLER_ID);
    private final ElevatorSubsystem elevator = new ElevatorSubsystem(MOTOR_ID);

    public RobotContainer() {
        new POVButton(controller, 90).whenPressed(new InstantCommand(() -> elevator.increaseHeight(), elevator));
        new POVButton(controller, 270).whenPressed(new InstantCommand(() -> elevator.decreaseHeight(), elevator));
    }
}