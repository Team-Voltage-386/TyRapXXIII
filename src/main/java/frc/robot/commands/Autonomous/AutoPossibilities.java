package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.PID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class AutoPossibilities extends CommandBase
{
    
    public static Command Choice (Piece StartingObject, Place StartingPlace, Position StartingLocation, Piece PickUpObject, Place PickedUpScore, Balancing Balance, boolean MartianRock)
    {
        if (!MartianRock)
        {
            //Martian Rock is false robot will run auto
            return new SequentialCommandGroup(PlaceObject(StartingObject, StartingPlace), BeginMove(StartingLocation, PickUpObject, Balance), 
            getPeice(PickUpObject), middleMove(StartingLocation, PickUpObject, Balance), autoBalance(Balance, StartingLocation), 
            returnRobot(StartingLocation, PickUpObject, Balance), PlaceObject(PickUpObject, PickedUpScore));
        }
        //Martian Rock is true robot won't do anything
        return null;
    }

    //Object place command for auto mode
    private static Command PlaceObject (Piece option, Place LMH)
    {
        if (LMH != Place.NULL)
        {
            /*Piece (CONE or CUBE) is what is being placed
            * LMH (LOW, MID or HIGH) is the position it is being placed in
            */
            if (option == Piece.CONE)
            {
                //Cone place 
                if (LMH == Place.HIGH)
                {
                    //Cone on high peg
                    //return
                }
                if (LMH == Place.MID)
                {
                    //Cone on middle peg
                    //return
                }
                //Cone on the ground (cone default if placement was not selected))
                //return
            }
            if (option == Piece.CUBE)
            {
                //Cube place 
                if (LMH == Place.HIGH)
                {
                    //Cube on high shelf
                    //return
                }
                if (LMH == Place.MID)
                {
                    //Cube on middle shelf
                    //return
                }
                //Cube on ground (cube default if placement was not selected)
                //return
            }
        }
        //Don't do anying (default if piece wasn't selected)
        return null;
    }

    //Command for the first half of the robot's move
    private static Command BeginMove (Position Starting, Piece pickingUp, Balancing balance)
    {
        if (Starting != null && balance != null)
        {
            //Robot has valid movement parameters

            //Cable starting position
            if (Starting == Position.CABLE && balance == Balancing.NORMAL && (pickingUp == null || pickingUp == Piece.NULL))
            {
                //Robot will move out of community from cable position, but not pick up a piece or balance
                //return
            }
            if (Starting == Position.CABLE && balance == Balancing.NORMAL && !(pickingUp == null || pickingUp == Piece.NULL))
            {
                //Robot will move out of community from cable position and balance, but not pick up a piece
                //return
            }
            if (Starting == Position.CABLE && balance == Balancing.BALANCE && (pickingUp == null || pickingUp == Piece.NULL))
            {
                //Robot will move out of community from cable position and balance, but not pick up a piece
                //return
            }
            if (Starting == Position.CABLE && balance == Balancing.BALANCE && !(pickingUp == null || pickingUp == Piece.NULL))
            {
                //Robot will move out of community from cable position, pick up a piece and balance
                //return
            }

            //Charger starting position
            if (Starting == Position.CHARGER && balance == Balancing.NORMAL && (pickingUp == null || pickingUp == Piece.NULL))
            {
                //Robot will move out of community from the charger stattion position, but not pick up a piece or balance
                //return
            }
            if (Starting == Position.CHARGER && balance == Balancing.NORMAL && !(pickingUp == null || pickingUp == Piece.NULL))
            {
                //Robot will move out of community from the charger stattion position and balance, but not pick up a piece
                //return
            }
            if (Starting == Position.CHARGER && balance == Balancing.BALANCE && (pickingUp == null || pickingUp == Piece.NULL))
            {
                //Robot will move out of community from the charger stattion position and balance, but not pick up a piece
                //return
            }
            if (Starting == Position.CHARGER && balance == Balancing.BALANCE && !(pickingUp == null || pickingUp == Piece.NULL))
            {
                //Robot will move out of community from the charger stattion position, pick up a piece and balance
                //return
            }

            //Clear starting position
            if (Starting == Position.CLEAR && balance == Balancing.NORMAL && (pickingUp == null || pickingUp == Piece.NULL))
            {
                //Robot will move out of community from clear position, but not pick up a piece or balance
                //return
            }
            if (Starting == Position.CLEAR && balance == Balancing.NORMAL && !(pickingUp == null || pickingUp == Piece.NULL))
            {
                //Robot will move out of community from clear position and balance, but not pick up a piece
                //return
            }
            if (Starting == Position.CLEAR && balance == Balancing.BALANCE && (pickingUp == null || pickingUp == Piece.NULL))
            {
                //Robot will move out of community from clear position and balance, but not pick up a piece
                //return
            }
            if (Starting == Position.CLEAR && balance == Balancing.BALANCE && !(pickingUp == null || pickingUp == Piece.NULL))
            {
                //Robot will move out of community from clear position, pick up a piece and balance
                //return
            }
        }
        //Martian Rock the robot does not have a starting position inputed
        return null;
    }

    private static Command middleMove(Position Starting, Piece PickedUp, Balancing Balance)
    {
        //Moves robot from piece to balance
        if ((PickedUp != null && PickedUp != Piece.NULL) && Balance == Balancing.BALANCE && Starting != null)
        {
            if (Starting == Position.CABLE)
            {
                //Manuver from cable piece to charger
                //return
            }

            if (Starting == Position.CHARGER)
            {
                //Manuver from charger piece to charger
                //return
            }

            if (Starting == Position.CLEAR)
            {
                //Manuver from clear piece to charger
                //return
            }
        }
        //Robot does not need to manuver to balance
        return null;
    }

    //Command for balancing 
    private static Command autoBalance(Balancing Balance, Position Starting)
    {
        if (Balance==Balancing.BALANCE && Starting!=null)
        {
            //Robot will balance
            //return 
        }
        //Robot won't balance
        return null;
    }

    //Command for robot picking up piece
    private static Command getPeice (Piece object)
    {
        if (object==Piece.CONE)
        {
            //Robot will pick up a cone
            //return
        }
        if (object==Piece.CUBE)
        {
            //Robot will pick up a cube
            //return
        }
        //robot does not pick up a piece 
        return null;
    }

    //Brings robot back to start
    private static Command returnRobot (Position Starting, Piece pickingUp, Balancing balance)
    {
        if (Starting != null && balance != null)
        {
            //Robot has valid movement parameters

            //Cable starting position
            if (Starting == Position.CABLE && balance == Balancing.NORMAL && (pickingUp == null || pickingUp == Piece.NULL))
            {
                //Robot will move in to the community at the cable position, but not pick up a piece or balance
                //return
            }
            if (Starting == Position.CABLE && balance == Balancing.NORMAL && !(pickingUp == null || pickingUp == Piece.NULL))
            {
                //Robot will move in to the community at the cable position and balance, but not pick up a piece
                //return
            }
            if (Starting == Position.CABLE && balance == Balancing.BALANCE && (pickingUp == null || pickingUp == Piece.NULL))
            {
                //Robot will move in to the community at the cable position and balance, but not pick up a piece
                //return
            }
            if (Starting == Position.CABLE && balance == Balancing.BALANCE && !(pickingUp == null || pickingUp == Piece.NULL))
            {
                //Robot will move in to the community at the cable position, pick up a piece and balance
                //return
            }

            //Charger starting position
            if (Starting == Position.CHARGER && balance == Balancing.NORMAL && (pickingUp == null || pickingUp == Piece.NULL))
            {
                //Robot will move in to the community at the the charger stattion position, but not pick up a piece or balance
                //return
            }
            if (Starting == Position.CHARGER && balance == Balancing.NORMAL && !(pickingUp == null || pickingUp == Piece.NULL))
            {
                //Robot will move in to the community at the the charger stattion position and balance, but not pick up a piece
                //return
            }
            if (Starting == Position.CHARGER && balance == Balancing.BALANCE && (pickingUp == null || pickingUp == Piece.NULL))
            {
                //Robot will move in to the community at the the charger stattion position and balance, but not pick up a piece
                //return
            }
            if (Starting == Position.CHARGER && balance == Balancing.BALANCE && !(pickingUp == null || pickingUp == Piece.NULL))
            {
                //Robot will move in to the community at the the charger stattion position, pick up a piece and balance
                //return
            }

            //Clear starting position
            if (Starting == Position.CLEAR && balance == Balancing.NORMAL && (pickingUp == null || pickingUp == Piece.NULL))
            {
                //Robot will move in to the community at the clear position, but not pick up a piece or balance
                //return
            }
            if (Starting == Position.CLEAR && balance == Balancing.NORMAL && !(pickingUp == null || pickingUp == Piece.NULL))
            {
                //Robot will move in to the community at the clear position and balance, but not pick up a piece
                //return
            }
            if (Starting == Position.CLEAR && balance == Balancing.BALANCE && (pickingUp == null || pickingUp == Piece.NULL))
            {
                //Robot will move in to the community at the clear position and balance, but not pick up a piece
                //return
            }
            if (Starting == Position.CLEAR && balance == Balancing.BALANCE && !(pickingUp == null || pickingUp == Piece.NULL))
            {
                //Robot will move in to the community at the clear position, pick up a piece and balance
                //return
            }
        }
        //Martian Rock the robot does not have a starting position inputed
        return null;
    }
}
