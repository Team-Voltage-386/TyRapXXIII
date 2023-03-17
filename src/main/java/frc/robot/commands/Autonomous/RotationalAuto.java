package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants.ArmSequences;
import frc.robot.RobotContainer;
import frc.robot.commands.ZeroOdo;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.Hand.handIntakeStates;
import frc.robot.utils.ArmKeyframe;

public class RotationalAuto extends CommandBase
{
    private Drivetrain DT;
    private Hand H;
    private Arm A;
    public RotationalAuto(Drivetrain dt, Hand h, Arm a)
    {
        DT=dt;
        H=h;
        A=a;
    }

    public Command getRotationalAuto(int Option)
    {
        //Code to place pieces and spin around on the sides
        //Made by Ryan for testing after the Orlando Regional
        if (Option==1)
        {
            return new SequentialCommandGroup(
                new ZeroOdo(0, 0, 0, DT),
                new SetConemode(true),
                new HandTasks(true, handIntakeStates.stow, H),
                new ArmDo(A, ArmSequences.kfseqConeStowToConeHigh),
                new HandTasks(false, handIntakeStates.doNothing, H),
                new ParallelCommandGroup(new ArmDo(A, ArmSequences.kfseqConeHightoCubeStow),
                new Drive(3, 135, 0, DT)),
                new Drive(3.5, 180, 0, DT));
        }
        if (Option==2)
        {
            return new SequentialCommandGroup(
                new ZeroOdo(0, 0, 0, DT),
                new HandTasks(false, handIntakeStates.stow, H),
                new SetConemode(false),
                new HandTasks(false, handIntakeStates.letitgo, H),
                new Drive(3, 135, 0, DT),
                new Drive(3.5, 180, 0, DT));
        }
        //Code to place pieces and spin around on the charger
        //Made by Ryan for testing during or after the Orlando Regional
        if (Option==3)
        {
            return new SequentialCommandGroup(
                new ZeroOdo(0, 0, 0, DT),
                new SetConemode(true),
                new HandTasks(true, handIntakeStates.stow, H),
                new ArmDo(A, ArmSequences.kfseqConeStowToConeHigh),
                new HandTasks(false, handIntakeStates.doNothing, H),
                new ParallelCommandGroup(new ArmDo(A, ArmSequences.kfseqConeHightoCubeStow),
                new Drive(0.5, 0, 180, DT),
                new Drive(2.5, 0, 180, DT)));
        }
        //Emergency state the code was called wrong basically the equivelent of a martian rock
        return new ZeroOdo(0, 0, 0, DT);
    }
}
