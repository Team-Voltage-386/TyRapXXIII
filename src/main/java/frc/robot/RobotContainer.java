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
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
    Piece StartingPiece = null, SecondPiece = null;
    Place FirstPlace = null, SecondPlace = null;
    Balancing Balance = null;
    Position StartingPosition =null;
    private static final ShuffleboardTab Auto = Shuffleboard.getTab("Auto");
    public final SendableChooser<Piece> StartingPieceChooser = new SendableChooser<>();
    public final SendableChooser<Place> StartingPlaceChooser = new SendableChooser<>();
    public final SendableChooser<Position> StartingPositionChooser = new SendableChooser<>();
    public final SendableChooser<Piece> SecondPieceChooser = new SendableChooser<>();
    public final SendableChooser<Place> SecondPlaceChooser = new SendableChooser<>();
    public final SendableChooser<Balancing> BalancingChooser = new SendableChooser<>();
    public final SendableChooser<Boolean> MartianRockChooser = new SendableChooser<>();

    
  public RobotContainer() {
    
    // Configure the trigger bindings
    configureBindings();

    StartingPieceChooser.addOption("Cone", Piece.CONE);
    StartingPieceChooser.addOption("Cube", Piece.CUBE);
    StartingPieceChooser.addOption("Nothing", Piece.NULL);
    Auto.add("Starting piece", StartingPieceChooser).withPosition(0, 0).withSize(1, 1);

    StartingPlaceChooser.addOption("High", Place.HIGH);
    StartingPlaceChooser.addOption("Middle", Place.MID);
    StartingPlaceChooser.addOption("Ground", Place.LOW);
    StartingPlaceChooser.addOption("Don't place", Place.NULL);
    Auto.add("Starting place", StartingPlaceChooser).withPosition(1, 0).withSize(1, 1);
        
    StartingPositionChooser.addOption("Cable strip side", Position.CABLE);
    StartingPositionChooser.addOption("Chager station", Position.CHARGER);
    StartingPositionChooser.addOption("Clear side", Position.CLEAR);
    Auto.add("Starting place", StartingPositionChooser).withPosition(2, 0).withSize(1, 1);
       
    SecondPieceChooser.addOption("Cone", Piece.CONE);
    SecondPieceChooser.addOption("Cube", Piece.CUBE);
    SecondPieceChooser.addOption("Nothing", Piece.NULL);
    Auto.add("Starting piece", SecondPieceChooser).withPosition(3, 0).withSize(1, 1);

    SecondPlaceChooser.addOption("High", Place.HIGH);
    SecondPlaceChooser.addOption("Middle", Place.MID);
    SecondPlaceChooser.addOption("Ground", Place.LOW);
    SecondPlaceChooser.addOption("Don't place", Place.NULL);
    Auto.add("Starting place", StartingPlaceChooser).withPosition(4, 0).withSize(1, 1);

    BalancingChooser.addOption("Balance", Balancing.BALANCE);
    BalancingChooser.addOption("Don't Balance", Balancing.NORMAL);
    Auto.add("Balanceing", StartingPlaceChooser).withPosition(5, 0).withSize(1, 1);
    
    MartianRockChooser.addOption("Martian Rock", true);
    MartianRockChooser.addOption("Normal", false);
    Auto.add("Martian Rock", MartianRockChooser).withPosition(6, 0).withSize(1, 1);
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
    return (new Drive(1, 1, 1, m_driveTrain));
    //return Choice();
  }
}
