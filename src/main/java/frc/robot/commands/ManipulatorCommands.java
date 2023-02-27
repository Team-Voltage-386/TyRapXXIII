// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.ControllerConstants.*;
import static frc.robot.Constants.DriveConstants.*;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Hand;
import frc.robot.utils.Flags;

public class ManipulatorCommands extends CommandBase {

  Hand HandControls;

  /** Creates a new ManipulatorCommands. */
  public ManipulatorCommands(Hand m_Hand) {
    HandControls = m_Hand;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Hand.updateWidget();
    //Mode switcher
    if (kManipulator.getRawButtonPressed(kLeftOptions)) 
    {
      Flags.ConeMode = false;
      Hand.ChangeMode();
    }
    if (kManipulator.getRawButtonPressed(kRightOptions)) 
    {
      Flags.ConeMode = true;
      Hand.ChangeMode();
    }

    //Cube picker-upper

    if(kManipulator.getRawButtonPressed(kY)) {
      Hand.IntakeMotorControl(true);
    }
    if(kManipulator.getRawButtonPressed(kA)) {
      Hand.IntakeMotorControl(false);
    }

    //Rotator
    if (kManipulator.getRawButtonPressed(kRightBumper) && Flags.canRotate) 
    {
      HandControls.RotateHand(true);
    }
    if (kManipulator.getRawButtonPressed(kLeftBumper) && Flags.canRotate) 
    {
      HandControls.RotateHand(false);
    }

    //Test Motor
    HandControls.setLimitClear();
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
