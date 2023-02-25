// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.AutoConstants.*;
import frc.robot.commands.DriverCommands;
import frc.robot.commands.Autonomous.Drive;
import frc.robot.commands.Autonomous.AutoPossibilities;
import frc.robot.commands.ManipulatorCommands;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.AutoShuffleBoard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
  private final Limelight m_ll = new Limelight();
  private final DriverCommands m_driverCommand = new DriverCommands(m_driveTrain);
  private final ManipulatorCommands m_manipulatorCommand = new ManipulatorCommands();
  

  // The robot's auto values are declared here
  Piece StartingPiece = null, SecondPiece = null;
  Place FirstPlace = null, SecondPlace = null;
  Balancing Balance = null;
  Position StartingPosition =null;
  boolean MartianRock = true;
  public static AutoShuffleBoard AutoValues;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
    
  public RobotContainer() 
  {
    // Configure the trigger bindings
    configureBindings();
    AutoValues = new AutoShuffleBoard();
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
  public Command getAutonomousCommand() 
  {
    AutoPossibilities autoMode = new AutoPossibilities();
    StartingPiece = AutoValues.getStartingPiece();
    FirstPlace = AutoValues.getStartingPlace();
    SecondPiece = AutoValues.getSecondPiece();
    SecondPlace = AutoValues.getSecondPlace();
    StartingPosition = AutoValues.getStartingPosition();
    Balance = AutoValues.getBalance();
    MartianRock = AutoValues.getMartianRock();
    return autoMode.Choice(StartingPiece, FirstPlace, StartingPosition, SecondPiece, SecondPlace, Balance, MartianRock);
  }
}
