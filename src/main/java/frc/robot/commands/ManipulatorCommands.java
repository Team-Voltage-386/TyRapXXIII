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
    ConeMode = true;
    manipulatorSetState = subsystemsStates.runStow;
  }

  public GenericEntry manipulationStateWidget = Shuffleboard.getTab("Main").add("manipState", "").getEntry();

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    manipulationStateWidget.setString(manipulatorSetState.toString());
    // pickup
    if (kManipulator.getRawButtonPressed(kY)) {
      manipulatorSetState = subsystemsStates.runPickup;
    }
    // score
    if (kManipulator.getRawButtonPressed(kA)) {
      manipulatorSetState = subsystemsStates.runScore;

    }
    // stow
    if (kManipulator.getRawButtonPressed(kB)) {
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

        switch (m_arm.lastKeyframe.keyFrameState) {
          case scoreCubeMid:
            m_arm.setKeyFrameSequence(kfseqCubeMidtoCubeStow);
            break;
          case scoreCubeHigh:
            m_arm.setKeyFrameSequence(kfseqCubehightoCubeStow);
            break;
          case scoreConeMid:
            m_arm.setKeyFrameSequence(kfseqConeMidtoCubeStow);
            break;
          case scoreConeHigh:
            m_arm.setKeyFrameSequence(kfseqConeHightoCubeStow);
            break;
          case pickup:
            m_arm.setKeyFrameSequence(kfseqCubePickuptoCubeStow);
            break;
          default:
            // do nothing
            break;
        }

        // arm do
        if (m_arm.nextKeyframe.keyFrameState == armKeyFrameStates.stowed) {
          if (!m_hand.canRetract()) {
            m_arm.runningKeyframesAndSequences = false;
          }
        } else {
          m_arm.runningKeyframesAndSequences = true;
        }
        // hand tasks
        // zero wrist
        m_hand.handPosition = 0;
        // intake tasks
        m_hand.IntakeMotorControl(handIntakeStates.doNothing);
        if (kManipulator.getRawButton(kX)) {
          m_hand.IntakeMotorControl(handIntakeStates.letitgo);
          m_hand.pcmCompressor.set(Value.kReverse);

        }
        break;
      case runPickup:
        // arm sequence

        m_arm.setKeyFrameSequence(kfseqCubeStowToCubePickup);
        // hand tasks
        // wrist position setter
        if (kManipulator.getRawButtonPressed(kRightBumper)) {
          m_hand.setRotateHand(true);
        }
        if (kManipulator.getRawButtonPressed(kLeftBumper)) {
          m_hand.setRotateHand(false);
        }
        // change elbow angle if wrist is rotated
        if (m_arm.lastKeyframe.keyFrameState == armKeyFrameStates.pickup) {
          if (m_hand.handPosition != 0) {
            m_arm.ElbowTarget = kElbowWristedPickup;
          } else {
            m_arm.ElbowTarget = kElbowPickupNormal;
          }
        }
        // intake motors
        if (kManipulator.getRawButton(kX)) {
          m_hand.IntakeMotorControl(handIntakeStates.letitgo);
          m_hand.pcmCompressor.set(Value.kReverse);

        } else if (m_arm.lastKeyframe.keyFrameState == armKeyFrameStates.pickup) {
          m_hand.IntakeMotorControl(handIntakeStates.intake);
        } else {
          m_hand.IntakeMotorControl(handIntakeStates.doNothing);
        }
        // mode switcher
        if (m_arm.lastKeyframe.keyFrameState != armKeyFrameStates.stowed) {
          if (kManipulator.getRawButtonPressed(kLeftOptions)) {
            ConeMode = false;
          }
          if (kManipulator.getRawButtonPressed(kRightOptions)) {
            ConeMode = true;
          }
          m_hand.ChangeMode();

        }
        break;

      case runScore:
        // store score position

        // set arm targets
        if (!ConeMode) {
          if (!scoreHighTarget) {
            m_arm.setKeyFrameSequence(kfseqCubeStowToCubeMid);
          } else {
            m_arm.setKeyFrameSequence(kfseqCubeStowToCubeHigh);
          }

        } else {
          if (!scoreHighTarget) {
            m_arm.setKeyFrameSequence(kfseqConeStowToConeMid);
          } else {
            m_arm.setKeyFrameSequence(kfseqConeStowToConeHigh);
          }

        }
        // hand tasks
        // intake motors
        if (kManipulator.getRawButton(kX)) {
          m_hand.IntakeMotorControl(handIntakeStates.letitgo);
          m_hand.pcmCompressor.set(Value.kReverse);
        } else {
          m_hand.IntakeMotorControl(handIntakeStates.doNothing);
        }
        // zero wrist
        m_hand.handPosition = 0;
        // arm tasks

        break;

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
