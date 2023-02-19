package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.AutoConstants.*;

public class AutoShuffleBoard
{
    //Initialization of all shuffleboard auto options
    private static final ShuffleboardTab Auto = Shuffleboard.getTab("Auto");
    public final SendableChooser<Piece> StartingPieceChooser = new SendableChooser<>();
    public final SendableChooser<Place> StartingPlaceChooser = new SendableChooser<>();
    public final SendableChooser<Position> StartingPositionChooser = new SendableChooser<>();
    public final SendableChooser<Piece> SecondPieceChooser = new SendableChooser<>();
    public final SendableChooser<Place> SecondPlaceChooser = new SendableChooser<>();
    public final SendableChooser<Balancing> BalancingChooser = new SendableChooser<>();
    //Martian rock is an exception to enums and will be using an integer value
    public final SendableChooser<Integer> MartianRockChooser = new SendableChooser<>();
    //
    public GenericEntry AutoReady;

    public AutoShuffleBoard()
    {
        //Adds all of the auto options to the auto shuffleboard
        StartingPieceChooser.addOption("Cone", Piece.CONE);
        StartingPieceChooser.addOption("Cube", Piece.CUBE);
        StartingPieceChooser.addOption("Nothing", Piece.NULL);
        Auto.add("Starting piece", StartingPieceChooser).withPosition(0, 0).withSize(2, 2);

        StartingPlaceChooser.addOption("High", Place.HIGH);
        StartingPlaceChooser.addOption("Middle", Place.MID);
        StartingPlaceChooser.addOption("Ground", Place.LOW);
        StartingPlaceChooser.addOption("Don't place", Place.NULL);
        Auto.add("Starting place", StartingPlaceChooser).withPosition(2, 0).withSize(2, 2);
            
        StartingPositionChooser.addOption("Cable strip side", Position.CABLE);
        StartingPositionChooser.addOption("Chager station", Position.CHARGER);
        StartingPositionChooser.addOption("Clear side", Position.CLEAR);
        Auto.add("Starting location", StartingPositionChooser).withPosition(4, 0).withSize(2, 2);
        
        SecondPieceChooser.addOption("Cone", Piece.CONE);
        SecondPieceChooser.addOption("Cube", Piece.CUBE);
        SecondPieceChooser.addOption("Nothing", Piece.NULL);
        Auto.add("Second piece", SecondPieceChooser).withPosition(0, 2).withSize(2, 2);

        SecondPlaceChooser.addOption("High", Place.HIGH);
        SecondPlaceChooser.addOption("Middle", Place.MID);
        SecondPlaceChooser.addOption("Ground", Place.LOW);
        SecondPlaceChooser.addOption("Don't place", Place.NULL);
        Auto.add("Second piece place", SecondPlaceChooser).withPosition(2, 2).withSize(2, 2);

        BalancingChooser.addOption("Balance", Balancing.BALANCE);
        BalancingChooser.addOption("Don't Balance", Balancing.NORMAL);
        Auto.add("Balanceing", BalancingChooser).withPosition(4, 2).withSize(2, 2);

        MartianRockChooser.addOption("Martian Rock Mode", 1);
        MartianRockChooser.setDefaultOption("Normal", 0);
        Auto.add("Martian Rock", MartianRockChooser).withPosition(6, 2).withSize(2, 2);

        Auto.add("Auto Mode is Ready", false).withPosition(0, 6).withSize(2, 2);
    }

    public Piece getStartingPiece ()
    {
        return StartingPieceChooser.getSelected();
    }

    public Place getStartingPlace ()
    {
        return StartingPlaceChooser.getSelected();
    }

    public Piece getSecondPiece ()
    {
        return SecondPieceChooser.getSelected();
    }

    public Place getSecondPlace ()
    {
        return SecondPlaceChooser.getSelected();
    }

    public Balancing getBalance ()
    {
        return BalancingChooser.getSelected();
    }

    public Position getStartingPosition ()
    {
        return StartingPositionChooser.getSelected();
    }

    public boolean getMartianRock ()
    {
        if (MartianRockChooser.getSelected()==1)
        {
            return true;
        }
        return false;
    }

    public void updateAutoReady()
    {
        boolean Ready = false;
        if (getStartingPiece()!=null && getStartingPlace()!=null && getStartingPosition()!=null && getBalance()!=null && 
        getSecondPiece()!=null && getSecondPlace()!=null && getMartianRock()==false)
        {
            Ready = true;
        }
        AutoReady.updateValue(Ready);
    }
}
