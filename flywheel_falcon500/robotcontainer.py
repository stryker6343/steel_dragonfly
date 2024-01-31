# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import commands2
from wpilib import SmartDashboard

from subsystems.flywheel_subsystem import Falcon500FlywheelSubsystem
import constants


class RobotContainer:
    """
    The robot container defines the overall structure of the robot.
    """

    def __init__(self):
        # The driver's controller
        self.driver_controller = commands2.button.CommandXboxController(
            constants.OIConstants.kDriverControllerPort
        )

        self.flywheel = Falcon500FlywheelSubsystem()

        # Configure the button bindings
        self.configureButtonBindings()

        SmartDashboard.putData("Flywheel", self.flywheel)

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        # Move the flywheel 50 rotations forward.
        self.driver_controller.a().onTrue(
            commands2.cmd.run(lambda: self.moveFlywheel(50), self.flywheel)
        )

        # Move the flywheel to the home position when the 'B' button is pressed
        self.driver_controller.b().onTrue(
            commands2.cmd.run(
                lambda: self.moveFlywheel(0),
                self.flywheel,
            )
        )

    def getAutonomousCommand(self) -> commands2.Command:
        """Returns the autonomous command"""
        return commands2.cmd.none()

    def moveFlywheel(self, rotations: int) -> None:
        """Moves the flywheel to the position specified by motor shaft rotations."""
        self.flywheel.move_to_position(rotations)
