// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.Hand.handIntakeStates;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.utils.Flags;
import frc.robot.utils.ArmKeyframe.armKeyFrameStates;
import frc.robot.utils.ArmKeyframe;
import frc.robot.RobotContainer;

import static frc.robot.Constants.ControllerConstants.*;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.utils.Flags.*;
import static frc.robot.Constants.ArmConstants.ArmSequences.*;

public class ManipulatorCommands extends CommandBase {
  private Arm m_arm;
  private Hand m_hand;
  private LEDSubsystem m_led;

  /** Creates a new ManipulatorCommands. */
  public ManipulatorCommands(Arm arm, Hand Hand, LEDSubsystem LEDSubsystem) {
    m_arm = arm;
    m_hand = Hand;
    m_led = LEDSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    scoreHighTarget = true;
    manipulatorSetState = subsystemsStates.runStow;
    manipulatorConemode = ConeMode;
  }

  public GenericEntry manipulationStateWidget = Shuffleboard.getTab("Main").add("manipState", "").getEntry();
  public boolean manipulatorConemode;
  public boolean manipulatorChutemode;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    manipulationStateWidget.setString(manipulatorSetState.toString());
    if (kManipulator.getRawButton(kRightBumper)) 
    {
      m_hand.TestPID(180);
      System.out.println("Hello World!");
    }
    else
    {
      m_hand.TestPID(0);
      System.out.println("Goodbye!");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
