// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.Hand.handIntakeStates;
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

  /** Creates a new ManipulatorCommands. */
  public ManipulatorCommands(Arm arm, Hand m_Hand) {
    m_arm = arm;
    m_hand = m_Hand;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    scoreHighTarget = true;
    ConeMode = false;
    manipulatorSetState = subsystemsStates.runStow;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // pickup
    if (kManipulator.getRawButtonPressed(kY)) {
      manipulatorSetState = subsystemsStates.runPickup;
    }
    // score
    if (kManipulator.getRawButtonPressed(kA)) {
      manipulatorSetState = subsystemsStates.runScore;

    }
    // stow
    if (kManipulator.getRawButton(kB)) {
      manipulatorSetState = subsystemsStates.runStow;
    }

    // score high, low, left, right, etc
    if (kManipulator.getPOV() == 0) {
      scoreHighTarget = true;
    }
    if (kManipulator.getPOV() == 180) {
      scoreHighTarget = false;
    }

    // // Test Motor
    // HandControls.setLimitClear();

    // total logic
    switch (manipulatorSetState) {

      case runStow:

        // arm do
        if (m_arm.nextKeyframe.keyFrameState != armKeyFrameStates.stowed && !m_arm.runningKeyframesAndSequences) {
          m_arm.setKeyFrameSequence(onlyIntermediary1(akfStowed));
          if (m_hand.canRetract()) {
            m_arm.runningKeyframesAndSequences = true;
          }
        }
        if (m_arm.nextKeyframe.keyFrameState == armKeyFrameStates.stowed && !m_hand.canRetract()) {
          m_arm.runningKeyframesAndSequences = false;
        }
        // hand tasks
        // zero wrist
        m_hand.handPosition = 0;
        // intake tasks
        if (m_arm.nextKeyframe.keyFrameState == armKeyFrameStates.stowed && !kManipulator.getRawButton(kX)) {
          m_hand.IntakeMotorControl(handIntakeStates.letitgo);
        } else {
          m_hand.IntakeMotorControl(handIntakeStates.doNothing);
        }

      case runPickup:
        // arm sequence
        if (m_arm.nextKeyframe.keyFrameState != armKeyFrameStates.pickup && !m_arm.runningKeyframesAndSequences) {
          switch (m_arm.lastKeyframe.keyFrameState) {
            case stowed:
              m_arm.setKeyFrameSequence(onlyIntermediary1(akfPickupGround));
              break;
            default:
              m_arm.setKeyFrameSequence(ONLYintermediary2(akfPickupGround));
              break;
          }
        }
        // hand tasks
        // wrist position setter
        if (kManipulator.getRawButtonPressed(kRightBumper)) {
          m_hand.setRotateHand(true);
        }
        if (kManipulator.getRawButtonPressed(kLeftBumper)) {
          m_hand.setRotateHand(false);
        }
        // intake motors
        if (!kManipulator.getRawButton(kX) && m_arm.lastKeyframe.keyFrameState != armKeyFrameStates.stowed) {
          m_hand.IntakeMotorControl(handIntakeStates.letitgo);
        } else if (m_arm.lastKeyframe.keyFrameState == armKeyFrameStates.pickup) {
          m_hand.IntakeMotorControl(handIntakeStates.intake);
        }
        // mode switcher

        if (m_arm.lastKeyframe.keyFrameState != armKeyFrameStates.stowed) {
          if (kManipulator.getRawButtonPressed(kLeftOptions)) {
            ConeMode = false;
            m_hand.ChangeMode();
          }
          if (kManipulator.getRawButtonPressed(kRightOptions)) {
            ConeMode = true;
            m_hand.ChangeMode();
          }
        }

      case runScore:
        // hand tasks
        // intake motors
        if (!kManipulator.getRawButton(kX) && m_arm.lastKeyframe.keyFrameState != armKeyFrameStates.stowed) {
          m_hand.IntakeMotorControl(handIntakeStates.letitgo);
        } else {
          m_hand.IntakeMotorControl(handIntakeStates.doNothing);
        }
        // zero wrist
        m_hand.handPosition = 0;
        // arm tasks
        ArmKeyframe end;
        // feed sequence
        if (m_arm.lastKeyframe.keyFrameState != armKeyFrameStates.pickup && !m_arm.runningKeyframesAndSequences) {
          if (ConeMode) {
            if (scoreHighTarget) {
              end = akfConeHigh;
            } else {
              end = akfConeMid;
            }
          } else {
            if (scoreHighTarget) {
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
