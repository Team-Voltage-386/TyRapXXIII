// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

import static frc.robot.Constants.ControllerConstants.*;

public class ManipulatorCommands extends CommandBase {
  private Arm arm;
  private double targetX,targetY;

  /** Creates a new ManipulatorCommands.
   * Teleop Manipulator controls
   */
  public ManipulatorCommands(Arm ARM) {
    arm=ARM;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetX=0.0;targetY=0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // targetX+=kManipulator.getRawAxis(kLeftHorizontal)*.1;
    // targetY+=kManipulator.getRawAxis(kLeftVertical)*.1;
    //increment targets
    if(kManipulator.getRawButtonPressed(kA))arm.ShoulderTarget-=15;
    if(kManipulator.getRawButtonPressed(kY))arm.ShoulderTarget+=15;
    if(kManipulator.getRawButtonPressed(kB))arm.ElbowTarget-=15;
    if(kManipulator.getRawButtonPressed(kX))arm.ElbowTarget+=15;
    // arm.ArmIKDrive(targetX, targetY);
    // arm.JoystickDriveRawArm(-kManipulator.getRawAxis(kRightVertical), -kManipulator.getRawAxis(kLeftVertical));//uncomment this to just drive motors using joysticks
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
