// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DriverCommands;
import frc.robot.commands.ManipulatorCommands;
import frc.robot.commands.WPI_Drive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.WPI_Drivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.ControllerConstants.*;
import static frc.robot.Constants.DriveConstants.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    public final WPI_Drivetrain m_driveTrain = new WPI_Drivetrain();
    private final Arm m_Arm = new Arm();
    private final Limelight m_ll = new Limelight();
    private final LEDSubsystem m_LED = new LEDSubsystem();

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        m_driveTrain.setDefaultCommand(new WPI_Drive(
                m_driveTrain,
                () -> -kDriver.getRawAxis(kLeftVertical),
                () -> kDriver.getRawAxis(kLeftHorizontal),
                () -> kDriver.getRawAxis(kRightHorizontal),
                () -> !(kDriver.getRawAxis(kRightTrigger) < 0.5))
        );

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        new JoystickButton(kDriver, kB).whenPressed(() -> m_driveTrain.zeroHeading());
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
