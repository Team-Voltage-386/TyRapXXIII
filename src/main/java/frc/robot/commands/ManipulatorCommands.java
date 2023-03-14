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
  public ManipulatorCommands(Arm arm, Hand Hand,LEDSubsystem LEDSubsystem) {
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
    manipulatorConemode=ConeMode;
  }

  public GenericEntry manipulationStateWidget = Shuffleboard.getTab("Main").add("manipState", "").getEntry();
    public boolean manipulatorConemode;
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
    
    if (kManipulator.getRawButtonPressed(kLeftOptions)) {
      manipulatorConemode= false;
    }
    if (kManipulator.getRawButtonPressed(kRightOptions)) {
      manipulatorConemode = true;
    }
    m_led.setLEDConeMode(manipulatorConemode);
    

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
        // if (m_arm.nextKeyframe.keyFrameState == armKeyFrameStates.stowed && m_arm.lastKeyframe.keyFrameState!=armKeyFrameStates.stowed) {
        //   if (!m_hand.canRetract()) {
        //     m_arm.runningKeyframesAndSequences = false;
        //     m_arm.ElbowTarget=75;
        //   } else {
        //     m_arm.runningKeyframesAndSequences = true;
        //   }
        // } 
        // hand tasks
        // zero wrist
        m_hand.handPosition = 0;
        // intake tasks
        m_hand.IntakeMotorControl(handIntakeStates.stow);
        if (kManipulator.getRawAxis(kRightTrigger) > kDeadband) {
          m_hand.IntakeMotorControl(handIntakeStates.letitgo);
          m_hand.pcmCompressor.set(Value.kReverse);

        }

        if(ConeMode){
          if (kManipulator.getRawAxis(kLeftTrigger) > kDeadband) {
            m_hand.pcmCompressor.set(Value.kReverse);
          }
          if(m_arm.lastKeyframe.keyFrameState == armKeyFrameStates.stowed && kManipulator.getRawAxis(kLeftTrigger) <= kDeadband) {
            m_hand.pcmCompressor.set(Value.kForward);
          }
        }

        if(m_arm.lastKeyframe.keyFrameState != armKeyFrameStates.pickup) {
          m_arm.dontProtectArm();
        }

        break;
      case runPickup:
        // arm sequence
        if (m_arm.lastKeyframe.keyFrameState == armKeyFrameStates.stowed) {
          m_arm.setKeyFrameSequence(kfseqCubeStowToCubePickup);
        }
        // hand tasks
        // wrist position setter
        if (kManipulator.getRawButtonPressed(kRightBumper) && ConeMode) {
          m_hand.setRotateHand(true);
        }
        if (kManipulator.getRawButtonPressed(kLeftBumper) && ConeMode) {
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
        if (kManipulator.getRawAxis(kRightTrigger) > kDeadband) {
          m_hand.IntakeMotorControl(handIntakeStates.letitgo);
          m_hand.pcmCompressor.set(Value.kReverse);

        } else if (m_arm.lastKeyframe.keyFrameState == armKeyFrameStates.pickup) {
          m_hand.IntakeMotorControl(handIntakeStates.intake);
        } else {
          m_hand.IntakeMotorControl(handIntakeStates.stow);
        }
        // mode switcher
        if (m_arm.lastKeyframe.keyFrameState != armKeyFrameStates.stowed) {
          ConeMode=manipulatorConemode;

          m_hand.ChangeMode();

        }

        if(ConeMode){
          if (kManipulator.getRawAxis(kLeftTrigger)>kDeadband) {
            m_hand.pcmCompressor.set(Value.kReverse);
          }
          if(kManipulator.getRawAxis(kLeftTrigger)<=kDeadband) {
            m_hand.pcmCompressor.set(Value.kForward);
          }
        }

        if(m_arm.lastKeyframe.keyFrameState == armKeyFrameStates.pickup && !m_arm.runningKeyframesAndSequences) {
          m_arm.protectArm();
        }

        break;

      case runScore:
        // store score position

        // set arm targets
        if (m_arm.lastKeyframe.keyFrameState == armKeyFrameStates.stowed) {
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
        }
        // hand tasks
        // intake motors
        if (kManipulator.getRawAxis(kRightTrigger) > kDeadband) {
          if (!ConeMode) {
            m_hand.IntakeMotorControl(handIntakeStates.letitgo);
          } else {
            m_hand.IntakeMotorControl(handIntakeStates.stow);
          }
          m_hand.pcmCompressor.set(Value.kReverse);
        } else {
          m_hand.IntakeMotorControl(handIntakeStates.stow);
        }
        // zero wrist
        m_hand.handPosition = 0;
        // arm tasks

        if(m_arm.lastKeyframe.keyFrameState != armKeyFrameStates.pickup) {
          m_arm.dontProtectArm();
        }

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
