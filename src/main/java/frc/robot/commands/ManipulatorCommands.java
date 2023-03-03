// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Hand;
import frc.robot.utils.Flags;
import frc.robot.utils.ArmKeyframe;
import frc.robot.RobotContainer;

import static frc.robot.Constants.ControllerConstants.*;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.utils.Flags.*;
import static frc.robot.Constants.ArmConstants.ArmSequences.*;

public class ManipulatorCommands extends CommandBase {
  private Arm m_arm;
  private Hand HandControls;

  /** Creates a new ManipulatorCommands. */
  public ManipulatorCommands(Arm arm, Hand m_Hand) {
    m_arm = arm;
    HandControls = m_Hand;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    scoreHigh = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Mode switcher
    if (kManipulator.getRawButtonPressed(kLeftOptions)) {
      ConeMode = false;
      HandControls.ChangeMode();
    }
    if (kManipulator.getRawButtonPressed(kRightOptions)) {
      ConeMode = true;
      HandControls.ChangeMode();
    }

    // pickup
    if (kManipulator.getRawButtonPressed(kY)) {
      // Hand.IntakeMotorControl(true);
      switch (m_arm.lastKeyframe.keyFrameState) {
        case stowed:
          m_arm.setKeyFrameSequence(onlyIntermediary1(akfPickupGround));

          break;
        default:
          m_arm.setKeyFrameSequence(ONLYintermediary2(akfPickupGround));
          break;
      }
    }
    // score
    if (kManipulator.getRawButtonPressed(kA)) {
      ArmKeyframe end;
      // Hand.IntakeMotorControl(false);
      if (ConeMode) {
        if (scoreHigh) {
          end = akfConeHigh;
        } else {
          end = akfConeMid;
        }
      } else {
        if (scoreHigh) {
          end = akfCubeHigh;
        } else {
          end = akfCubeMid;
        }
      }

      switch (m_arm.lastKeyframe.keyFrameState) {
        case stowed:
          m_arm.setKeyFrameSequence(fromStowGoTo(end));
          break;
        default:
          ONLYintermediary2(end);
          break;
      }
    }
    // stow
    if (kManipulator.getRawButton(kB)) {
      m_arm.setKeyFrameSequence(onlyIntermediary1(akfStowed));
    }

    // score high, low, left, right, etc
    if (kManipulator.getPOV() == 0) {
      scoreHigh = true;
    }
    if (kManipulator.getPOV() == 180) {
      scoreHigh = false;
    }

    // Cube picker-upper

    if (kManipulator.getRawButtonPressed(kY)) {
      HandControls.IntakeMotorControl(true);
    }
    if (kManipulator.getRawButtonPressed(kA)) {
      HandControls.IntakeMotorControl(false);
    }

    // Rotator
    if (kManipulator.getRawButtonPressed(kRightBumper) && Flags.canRotate) {
      HandControls.RotateHand(true);
    }
    if (kManipulator.getRawButtonPressed(kLeftBumper) && Flags.canRotate) {
      HandControls.RotateHand(false);
    }

    // Test Motor
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
