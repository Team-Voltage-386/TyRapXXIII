// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.utils.ArmKeyframe;
import static frc.robot.utils.Flags.*;

public class ArmDo extends CommandBase {
  private Arm m_arm;
  private ArmKeyframe[] m_sequence;
  /**
   * set arm sequence of arm
   * @param arm
   * @param sequence
   */
  public ArmDo(Arm arm, ArmKeyframe[] sequence) {
    m_arm=arm;
    m_sequence=sequence;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.setKeyFrameSequence(m_sequence);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return armIsAtTarget;
  }
}
