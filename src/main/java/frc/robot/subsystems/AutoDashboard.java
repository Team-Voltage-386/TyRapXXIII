package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.Subscriber;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Robot;
import frc.robot.Constants.AutoConstants.*;

public class AutoDashboard extends Robot
{
    Piece StartingPiece = null, SecondPiece = null;
    Place FirstPlace = null, SecondPlace = null;
    Balancing Balance = null;
    Position StartingPosition =null;
    private static final ShuffleboardTab Auto = Shuffleboard.getTab("Auto");
    public final SendableChooser<Piece> StartingPieceChooser = new SendableChooser<>();
    public final SendableChooser<Place> StartingPlaceChooser = new SendableChooser<>();
    public final SendableChooser<Position> StartingPositionChooser = new SendableChooser<>();
    public final SendableChooser<Piece> SecondPieceChooser = new SendableChooser<>();
    public final SendableChooser<Place> SecondPlaceChooser = new SendableChooser<>();
    public final SendableChooser<Balancing> BalancingChooser = new SendableChooser<>();
    public final SendableChooser<Boolean> MartianRockChooser = new SendableChooser<>();

    @Override
    public void autonomousInit()
    {
        StartingPieceChooser.addOption("Cone", Piece.CONE);
        StartingPieceChooser.addOption("Cube", Piece.CUBE);
        StartingPieceChooser.addOption("Nothing", Piece.NULL);
        Auto.add("Starting piece", StartingPieceChooser).withPosition(0, 0).withSize(1, 1);

        StartingPlaceChooser.addOption("High", Place.HIGH);
        StartingPlaceChooser.addOption("Middle", Place.MID);
        StartingPlaceChooser.addOption("Ground", Place.LOW);
        StartingPlaceChooser.addOption("Don't place", Place.NULL);
        Auto.add("Starting place", StartingPlaceChooser).withPosition(1, 0).withSize(1, 1);
        
        StartingPositionChooser.addOption("Cable strip side", Position.CABLE);
        StartingPositionChooser.addOption("Chager station", Position.CHARGER);
        StartingPositionChooser.addOption("Clear side", Position.CLEAR);
        Auto.add("Starting place", StartingPositionChooser).withPosition(2, 0).withSize(1, 1);
        
        SecondPieceChooser.addOption("Cone", Piece.CONE);
        SecondPieceChooser.addOption("Cube", Piece.CUBE);
        SecondPieceChooser.addOption("Nothing", Piece.NULL);
        Auto.add("Starting piece", SecondPieceChooser).withPosition(3, 0).withSize(1, 1);

        SecondPlaceChooser.addOption("High", Place.HIGH);
        SecondPlaceChooser.addOption("Middle", Place.MID);
        SecondPlaceChooser.addOption("Ground", Place.LOW);
        SecondPlaceChooser.addOption("Don't place", Place.NULL);
        Auto.add("Starting place", StartingPlaceChooser).withPosition(4, 0).withSize(1, 1);

        BalancingChooser.addOption("Balance", Balancing.BALANCE);
        BalancingChooser.addOption("Don't Balance", Balancing.NORMAL);
        Auto.add("Balanceing", StartingPlaceChooser).withPosition(5, 0).withSize(1, 1);
    
        MartianRockChooser.addOption("Martian Rock", true);
        MartianRockChooser.addOption("Normal", false);
        Auto.add("Martian Rock", MartianRockChooser).withPosition(6, 0).withSize(1, 1);   
    }
    //private static final GenericEntry prepared =  Auto.add("Ready", false).withPosition(6, 0).withSize(1, 1);
    public void updateWidget()
    {
        //prepared.set(true);
    }
}
