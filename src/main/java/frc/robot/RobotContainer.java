// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.DriverCommands;
import frc.robot.commands.Autonomous.Balance;
import frc.robot.commands.Autonomous.Drive;
import frc.robot.commands.Autonomous.DriveUntilAngleDec;
import frc.robot.commands.Autonomous.DriveUntilAngleInc;
import frc.robot.commands.ManipulatorCommands;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.SwerveModule;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
  public final Drivetrain m_driveTrain = new Drivetrain();
  private final Arm m_Arm = new Arm();
  private final LimeLight m_ll = new LimeLight();
  private final DriverCommands m_driverCommand = new DriverCommands(m_driveTrain);
  private final ManipulatorCommands m_manipulatorCommand = new ManipulatorCommands();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
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

  public Command getTeleOp() {
    return m_driverCommand;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return new SequentialCommandGroup(new Drive(1, 0, 0, m_driveTrain), new
    // Drive(1, 1, 0, m_driveTrain),
    // new Drive(0, 1, 0, m_driveTrain), new Drive(0, 0, 0, m_driveTrain)); 
    //return new Drive(10, 0, 0, m_driveTrain);s
    //2.24
    //game autos
    //return new SequentialCommandGroup(new DriveUntilAngleInc(2.3, 0, 0, m_driveTrain, 10, 2), new Drive(3.8, 0, 0, m_driveTrain),new DriveUntilAngleInc(2.5, 0, 0, m_driveTrain, 8, 2), new Balance(m_driveTrain));
    //return new SequentialCommandGroup(new Drive(2.12, 0, 0, m_driveTrain), new Drive(4.24, 0, 0, m_driveTrain), new Drive(2.12, 0, 0, m_driveTrain));
    return new SequentialCommandGroup(new DriveUntilAngleInc(2, 0, 0, m_driveTrain, 10, 2), new Balance(m_driveTrain));
  }
}
