// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import static frc.robot.Constants.ControllerConstants.*;
import static frc.robot.utils.Flags.*;
public class ManipulatorCommands extends CommandBase {
  private Arm m_arm;

  /** Creates a new ManipulatorCommands. */
  public ManipulatorCommands(Arm arm) {
    m_arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Mode switcher
    if (kManipulator.getRawButtonPressed(kLeftOptions)) {
      ConeMode = false;
      // Hand.ChangeMode();
    }
    if (kManipulator.getRawButtonPressed(kRightOptions)) {
      ConeMode = true;
      // Hand.ChangeMode();
    }

    // Cube picker-upper

    if (kManipulator.getRawButtonPressed(kY)) {
      // Hand.IntakeMotorControl(true);
    }
    if (kManipulator.getRawButtonPressed(kA)) {
      // Hand.IntakeMotorControl(false);
    }

    // Rotator
    if (kManipulator.getRawButtonPressed(kRightBumper)) {
      // Hand.handPosition++;
      // if (Hand.handPosition > 1) {
      //   Hand.handPosition = 1;
      // } else {
      //   HandControls.RotateHand();
      // }
    }
    if (kManipulator.getRawButtonPressed(kLeftBumper)) {
      // Hand.handPosition--;
      // if (Hand.handPosition < -1) {
      //   Hand.handPosition = -1;
      // } else {
      //   HandControls.RotateHand();
      // }
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
