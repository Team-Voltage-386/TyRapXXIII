// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LEDSubsystem;

import static frc.robot.utils.Flags.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetConemode extends InstantCommand {
  private boolean m_ConeMode,m_doPressDown;
  private LEDSubsystem m_LED;
  public SetConemode(boolean coneMode, LEDSubsystem LED) {
    m_ConeMode=coneMode;
    m_LED=LED;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public SetConemode(boolean coneMode, boolean DoPressDown) {
    m_ConeMode = coneMode;
    m_doPressDown = DoPressDown;
  }
  public SetConemode(boolean coneMode) {
    m_ConeMode = coneMode;
    m_doPressDown = true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ConeMode = m_ConeMode;
    doPressDown = m_doPressDown;
    if(m_LED!=null){
      m_LED.setLEDConeMode(ConeMode);
    }
    
  }
}
