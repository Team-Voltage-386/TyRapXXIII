//A field tag defines a tag, using its id, within field space as an xyz coordinate. 
// x spans from red to blue (so we dont care here), y is the line drawn through the april tags. Z is height obviously.
package frc.robot.constantsHelpers;

public class FieldTag {
    public static double x,y,z,yaw;
    public static int id;
    public static boolean isGrid;
    
    public FieldTag(int ID,double X, double Y, double Z, boolean IsGrid, double Yaw){
        id=ID;
        x=X;
        y=Y;
        z=Z;
        isGrid=IsGrid;
        yaw=Yaw;
    }

}