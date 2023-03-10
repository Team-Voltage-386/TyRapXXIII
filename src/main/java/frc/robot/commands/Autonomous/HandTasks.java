// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Hand.handIntakeStates;
import frc.robot.subsystems.Hand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HandTasks extends InstantCommand {
  private Hand m_hand;
  private handIntakeStates m_IntakeState;
  private boolean m_clawClose;

  /**
   * 
   * @param clawClose
   * @param intakeDo
   * @param hand
   */
  public HandTasks(boolean clawClose, handIntakeStates intakeDo, Hand hand) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_hand = hand;
    m_IntakeState = intakeDo;
    addRequirements(hand);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_hand.IntakeMotorControl(m_IntakeState);
    if (m_clawClose) {
      m_hand.pcmCompressor.set((Value.kForward));
    } else {
      m_hand.pcmCompressor.set((Value.kReverse));
    }

  }
}
