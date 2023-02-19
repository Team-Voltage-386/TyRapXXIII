// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.Constants.LimelightConstants;

public class ManipulatorCommands extends CommandBase {
  private Limelight limelight;
  private Drivetrain drivetrain;

  /** Creates a new ManipulatorCommands. */
  public ManipulatorCommands(Limelight ll, Drivetrain dt) {
    limelight = ll;
    drivetrain = dt;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (limelight.apriltagsAvailable()) {
      drivetrain.overridePosition(39.37008*(limelight.getPose()[0]+8.270494), 39.37008*(limelight.getPose()[1]+4.008216));
      drivetrain.overrideOrientation(limelight.getPose()[5]);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
