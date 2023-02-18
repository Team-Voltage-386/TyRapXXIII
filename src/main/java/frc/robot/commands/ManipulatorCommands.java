// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Constants.ControllerConstants.*;

public class ManipulatorCommands extends CommandBase {
  private double targetX,targetY;

  /** Creates a new ManipulatorCommands. */
  public ManipulatorCommands() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    updateWidgets();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  private ShuffleboardTab mainTab=Shuffleboard.getTab("Main");
  private GenericPublisher joystickLeftAxisVertWidget=mainTab.add("joystickLeftAxisVert",0.0).withPosition(0, 2).withSize(1, 1).getEntry();
  private GenericPublisher joystickRightAxisVertWidget=mainTab.add("joystickRightAxisVert",0.0).withPosition(1, 2).withSize(1, 1).getEntry();
  private void updateWidgets(){
    joystickLeftAxisVertWidget.setDouble(kManipulator.getRawAxis(kLeftVertical));
    joystickRightAxisVertWidget.setDouble(kManipulator.getRawAxis(kRightVertical));

  }
}
