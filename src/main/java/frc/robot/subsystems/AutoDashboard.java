package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.Subscriber;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.AutoConstants.*;

public class AutoDashboard 
{
    Piece StartingPiece, SecondPiece;
    Place FirPlace, SecondPlace;
    Balancing Balance;
    Position StartingPosition;
    public final SendableChooser<Piece> StartingPieceChooser = new SendableChooser<>();
    public final SendableChooser<Place> StartingPlaceChooser = new SendableChooser<>();
    public final SendableChooser<Position> StartingPositionChooser = new SendableChooser<>();
  
    
    private static final ShuffleboardTab mainTab = Shuffleboard.getTab("Auto");
    private static final Subscriber ChooseStartPiece = mainTab.add("", 0).withPosition(0, 0).withSize(1, 1).getEntry();
    private static final GenericEntry yPosWidget = mainTab.add("Y", 0).withPosition(1, 0).withSize(1, 1).getEntry();
    private static final GenericEntry hPosWidget = mainTab.add("H", 0).withPosition(2, 0).withSize(1, 1).getEntry();

    private void updateWidget() 
    {
        
    }
}
