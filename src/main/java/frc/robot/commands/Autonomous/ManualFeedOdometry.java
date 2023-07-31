// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ManualFeedOdometry extends InstantCommand {
    public Drivetrain m_Drivetrain;
    private double x, y, h;

    public ManualFeedOdometry(Drivetrain drivetrain, double X, double Y, double H) {
        x = X;
        y = Y;
        h = H;
        m_Drivetrain = drivetrain;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_Drivetrain.feedBotPose(x, y, h);
    }
}
