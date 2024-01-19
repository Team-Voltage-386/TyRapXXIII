// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.DriverCommands;
import frc.robot.commands.Autonomous.Drive;
import frc.robot.commands.Autonomous.DriveUntil;
import frc.robot.commands.Autonomous.DriveAtSpeed;
import frc.robot.commands.Autonomous.DriveUntilAngleDec;
import frc.robot.commands.Autonomous.DriveUntilAngleInc;
import frc.robot.commands.Autonomous.HandTasks;
import frc.robot.commands.Autonomous.LogicBalance;
import frc.robot.commands.Autonomous.ManualFeedOdometry;
import frc.robot.commands.Autonomous.SetConemode;
import frc.robot.commands.Autonomous.ArmDo;
import frc.robot.commands.Autonomous.Balance;
import frc.robot.commands.ManipulatorCommands;
import frc.robot.commands.ZeroOdo;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Hand.handIntakeStates;
// import frc.robot.utils.AllianceData;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.Constants.ArmConstants.ArmSequences.*;
import javax.swing.plaf.TreeUI;

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
    private final LEDSubsystem m_LED = new LEDSubsystem();

    private final DriverCommands m_driverCommand = new DriverCommands(m_driveTrain);
    public final Hand HandControls = new Hand();
    private final ManipulatorCommands m_manipulatorCommand = new ManipulatorCommands(m_Arm, HandControls, m_LED);
    private final ParallelCommandGroup m_teleop = new ParallelCommandGroup(m_driverCommand, m_manipulatorCommand);

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    private final AutoRoutines autos = this.new AutoRoutines();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
        // autoChooser.addOption("test1", autos.test1);
        autoChooser.addOption("Score cone high and backup", autos.ScoreConeSides);
        autoChooser.addOption("Score cone high and backup onto the the charger", autos.ScoreConeCharger);
        autoChooser.addOption("Score cube low and backup", autos.ScoreCubeSides);
        // autoChooser.addOption("test3", autos.test3);
        autoChooser.addOption("test4", autos.test4);
        autoChooser.addOption("Logic Balance FACE FORWARD", autos.logicBalance);
        autoChooser.addOption("Drive Until", autos.driveUntil);
        autoChooser.addOption("Place Cube Low and Balance", autos.placeAndBalance);
        autoChooser.addOption("TuningSquare", autos.TuningSquare);
        autoChooser.addOption("Place Cone High and Cross Line", autos.placeAndCrossLine);
        autoChooser.addOption("Place Cube Low and Balance No Mobility", autos.placeAndBalanceNoMobility);
        // autoChooser.addOption("Middle Auto", autos.test1);
        Shuffleboard.getTab("Main").add("AutoRoutine", autoChooser).withSize(3, 1).withPosition(4, 2);
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

    public final class AutoRoutines {

        // Code for balancing
        /** drives over the drive station, comes back, and balances. By Lucas */
        public final Command test1 = new SequentialCommandGroup(
                new ZeroOdo(0, 0, 0, m_driveTrain),
                new DriveAtSpeed(4.2, 0, 0, 0.15, m_driveTrain),
                new DriveUntilAngleInc(1.9, 0, 180, 0.2, m_driveTrain, 9, 2),
                new DriveAtSpeed(2.345, 0, 0, 0.2, m_driveTrain));

        // Code for running on the sides
        public final Command test2 = new SequentialCommandGroup(
                new ZeroOdo(0, 0, 0, m_driveTrain),
                new HandTasks(true, handIntakeStates.stow, HandControls),
                new ArmDo(m_Arm, kfseqConeStowToConeHigh),
                new HandTasks(false, handIntakeStates.doNothing, HandControls),
                new ArmDo(m_Arm, kfseqConeHightoCubeStow),
                new Drive(3.8, 0, 0, m_driveTrain));
        public final Command ScoreConeSides = new SequentialCommandGroup(
                new ZeroOdo(0, 0, 0, m_driveTrain),
                new SetConemode(true),
                new HandTasks(true, handIntakeStates.stow, HandControls),
                new ArmDo(m_Arm, kfseqConeStowToConeHigh),
                new HandTasks(false, handIntakeStates.doNothing, HandControls),
                new ParallelCommandGroup(new ArmDo(m_Arm, kfseqConeHightoCubeStow),
                        new Drive(3, 0, 0, m_driveTrain)),
                new Drive(3.8, 0, 0, m_driveTrain));
        public final Command ScoreConeCharger = new SequentialCommandGroup(
                new ZeroOdo(0, 0, 0, m_driveTrain),
                new SetConemode(true),
                new HandTasks(true, handIntakeStates.stow, HandControls),
                new ArmDo(m_Arm, kfseqConeStowToConeHigh),
                new HandTasks(false, handIntakeStates.doNothing, HandControls),
                new ParallelCommandGroup(new ArmDo(m_Arm, kfseqConeHightoCubeStow),
                        new Drive(2.5, 0, 180, m_driveTrain)));
        public final Command ScoreCubeSides = new SequentialCommandGroup(
                new ZeroOdo(0, 0, 0, m_driveTrain),
                new HandTasks(false, handIntakeStates.stow, HandControls),
                new SetConemode(false),
                new HandTasks(false, handIntakeStates.letitgo, HandControls),
                new Drive(3, 0, 0, m_driveTrain),
                new Drive(3.8, 0, 0, m_driveTrain));

        // public final Command test3 = new SequentialCommandGroup(
        // new ManualFeedOdometry(m_driveTrain, 0, 0,
        // (AllianceData.resetOrientationOffset + 180) % 360),
        // new HandTasks(true, handIntakeStates.stow, HandControls),
        // new ArmDo(m_Arm, kfseqConeStowToConeHigh),
        // new HandTasks(false, handIntakeStates.doNothing, HandControls),
        // new ParallelCommandGroup(new ArmDo(m_Arm, kfseqConeHightoCubeStow),
        // new Drive(3 * AllianceData.fieldSideMultiplier, 0,
        // (AllianceData.resetOrientationOffset + 180) % 360, m_driveTrain)));
        public final Command test4 = new SequentialCommandGroup(
                new Drive(-3, 0, 0, m_driveTrain),
                new Drive(-3, -2, 0, m_driveTrain));
        public final Command logicBalance = new SequentialCommandGroup(
                new DriveUntil(true, m_driveTrain),
                new LogicBalance(m_driveTrain));
        public final Command driveUntil = new SequentialCommandGroup(
                new DriveUntil(false, m_driveTrain));
        public final Command placeAndBalance = new SequentialCommandGroup(
                new ZeroOdo(0, 0, 0, m_driveTrain),

                // new HandTasks(true, handIntakeStates.stow, HandControls),//cone high
                // new ArmDo(m_Arm, kfseqConeStowToConeHigh),
                // new HandTasks(false, handIntakeStates.doNothing, HandControls),
                // new ParallelCommandGroup(new ArmDo(m_Arm, kfseqConeHightoCubeStow),
                new HandTasks(false, handIntakeStates.stow, HandControls), // cube spit
                new SetConemode(false),
                new HandTasks(false, handIntakeStates.letitgo, HandControls),
                new Drive(4.1, 0, 0, m_driveTrain),
                new HandTasks(false, handIntakeStates.stow, HandControls),
                new DriveUntil(false, m_driveTrain),
                new LogicBalance(m_driveTrain));
        public final Command placeAndCrossLine = new SequentialCommandGroup(
                new HandTasks(true, handIntakeStates.stow, HandControls),
                new ArmDo(m_Arm, kfseqConeStowToConeHigh),
                new HandTasks(false, handIntakeStates.doNothing, HandControls),
                new ParallelCommandGroup(new ArmDo(m_Arm, kfseqConeHightoCubeStow),
                        new Drive(4.1, 0, 0, m_driveTrain)),
                new Drive(1, 0, 180, m_driveTrain));
        public final Command placeAndBalanceNoMobility = new SequentialCommandGroup(
                new ZeroOdo(0, 0, 0, m_driveTrain),
                new HandTasks(false, handIntakeStates.stow, HandControls),
                new SetConemode(false),
                new HandTasks(false, handIntakeStates.letitgo, HandControls),
                // new HandTasks(true, handIntakeStates.stow, HandControls),
                // new ArmDo(m_Arm, kfseqConeStowToConeHigh),
                // new HandTasks(false, handIntakeStates.doNothing, HandControls),
                // new ParallelCommandGroup(new ArmDo(m_Arm, kfseqConeHightoCubeStow),
                // new Drive(1, 0, 0, m_driveTrain)),
                new DriveUntil(true, m_driveTrain),
                new HandTasks(false, handIntakeStates.stow, HandControls),
                new LogicBalance(m_driveTrain));

        public final Command TuningSquare = new SequentialCommandGroup(
                new Drive(2, 0, 0, m_driveTrain),
                new Drive(2, 2, 90, m_driveTrain),
                new Drive(0, 2, 270, m_driveTrain),
                new Drive(0, 0, 90, m_driveTrain));
        public final Command spitOutCube = new SequentialCommandGroup(
                new SetConemode(false),
                new HandTasks(false, handIntakeStates.letitgo, HandControls));
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
