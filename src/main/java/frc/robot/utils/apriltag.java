package frc.robot.utils;
public class apriltag {
    public int id;
    public boolean isGrid,isRed;
    public double x,y,z;
    public apriltag(int ID, boolean IsGrid, boolean IsRed,double X, double Y, double Z){
        id=ID;
        isGrid=IsGrid;
        isRed=IsRed;
        x=X;
        y=Y;
        z=Z;
    }
}
