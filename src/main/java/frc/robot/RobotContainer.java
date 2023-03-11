// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.DriverCommands;
import frc.robot.commands.Autonomous.Drive;
import frc.robot.commands.Autonomous.HandTasks;
import frc.robot.commands.Autonomous.ArmDo;
import frc.robot.commands.ManipulatorCommands;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Hand.handIntakeStates;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.Constants.ArmConstants.ArmSequences.*;

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
  public final Hand HandControls = new Hand();
  private final ManipulatorCommands m_manipulatorCommand = new ManipulatorCommands(m_Arm, HandControls);
  private final ParallelCommandGroup m_teleop = new ParallelCommandGroup(m_driverCommand, m_manipulatorCommand);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  private final AutoRoutines autos = this.new AutoRoutines();

  public final LEDSubsystem LED = new LEDSubsystem();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    autoChooser.addOption("test1", autos.test1);
    autoChooser.addOption("test2", autos.test2);

    Shuffleboard.getTab("Main").add("AutoRoutine",autoChooser).withSize(3,1);

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
    return m_teleop;
  }

  //all auto routines go here, make sure to add to sendable chooseer
  public final class AutoRoutines {

    public final Command test1 = new SequentialCommandGroup(
        new HandTasks(true, handIntakeStates.stow, HandControls),
        new ArmDo(m_Arm, kfseqConeStowToConeHigh),
        new HandTasks(false, handIntakeStates.doNothing, HandControls),
        new ArmDo(m_Arm, kfseqConeHightoCubeStow),
        new Drive(-3, 0, 0, m_driveTrain)
    );
    public final Command test2 = new SequentialCommandGroup(
        new HandTasks(true, handIntakeStates.stow, HandControls),
        new ArmDo(m_Arm, kfseqConeStowToConeHigh),
        new HandTasks(false, handIntakeStates.doNothing, HandControls),
        new ParallelCommandGroup(new ArmDo(m_Arm, kfseqConeHightoCubeStow),
        new Drive(-3, 0, 0, m_driveTrain))
        );
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
