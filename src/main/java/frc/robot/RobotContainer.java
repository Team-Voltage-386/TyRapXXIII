// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.DriverCommands;
import frc.robot.commands.Autonomous.LucasPIDBalance;
import frc.robot.commands.Autonomous.Drive;
import frc.robot.commands.Autonomous.LogicBalance;
import frc.robot.commands.Autonomous.DriveUntil;
import frc.robot.commands.Autonomous.DriveUntilAngleDec;
import frc.robot.commands.Autonomous.DriveUntilAngleInc;
import frc.robot.commands.ManipulatorCommands;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.SwerveModule;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.DriverStation;

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
  private final Drivetrain m_driveTrain = new Drivetrain();
  private final Limelight m_ll = new Limelight();
  private final DriverCommands m_driverCommand = new DriverCommands(m_driveTrain, m_ll);
  private final Arm m_Arm = new Arm();
  private final ManipulatorCommands m_manipulatorCommand = new ManipulatorCommands(m_Arm);

  private final Command drive5 = new Drive(5, 0, 0, m_driveTrain);
  private final Command balance = new LogicBalance(m_driveTrain);
  private final SequentialCommandGroup goOverAndBalance = new SequentialCommandGroup(new DriveUntil(true, m_driveTrain),
      new Drive(5, 0, 0, m_driveTrain), new DriveUntil(false, m_driveTrain), new LogicBalance(m_driveTrain));

  // private static final Shuffleboard Tab mainTab = Shuffleboard.getTab("Main");

  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  public RobotContainer() {
    m_chooser.addOption("Drive to (5, 0)", drive5);
    m_chooser.addOption("Balance", balance);
    m_chooser.setDefaultOption("Go over and Balance", goOverAndBalance);

    SmartDashboard.putData(m_chooser);
    // mainTab.add("autoRoutine", m_chooser).withPosition(5, 5).withSize(3, 1);
    // Configure the trigger bindings
    configureBindings();
    m_driveTrain.setDefaultCommand(m_driverCommand);
    m_Arm.setDefaultCommand(m_manipulatorCommand);
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

  // public Command getTeleOp() {
  // return new ParallelCommandGroup(m_driverCommand,m_autopilotCOmmand);
  // return m_driverCommand;
  // }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //gabe logic balance
    // return new SequentialCommandGroup(new DriveUntil(m_driveTrain), new
    // Balance(m_driveTrain));
    // return new SequentialCommandGroup(new DriveUntil(true, m_driveTrain), new
    // Drive(5, 0, 0, m_driveTrain),
    // new DriveUntil(false, m_driveTrain),
    // new Balance(m_driveTrain));
    //return m_chooser.getSelected();
    
    //lucas PID balance 
    // return new SequentialCommandGroup(new Drive(1, 0, 0, m_driveTrain), new
    // Drive(1, 1, 0, m_driveTrain),
    // new Drive(0, 1, 0, m_driveTrain), new Drive(0, 0, 0, m_driveTrain));
    // return new Drive(10, 0, 0, m_driveTrain);s
    // 2.24
    // game autos
    // return new SequentialCommandGroup(new DriveUntilAngleInc(2.3, 0, 0,
    // m_driveTrain, 10, 2), new Drive(3.8, 0, 0, m_driveTrain),new
    // DriveUntilAngleInc(2.5, 0, 0, m_driveTrain, 8, 2), new
    // Balance(m_driveTrain));
    // return new SequentialCommandGroup(new Drive(2.12, 0, 0, m_driveTrain), new
    // Drive(4.24, 0, 0, m_driveTrain), new Drive(2.12, 0, 0, m_driveTrain));
    // return new SequentialCommandGroup(new DriveUntilAngleInc(2, 0, 0, m_driveTrain, 10, 2),new LucasPIDBalance(m_driveTrain));
  }
}
