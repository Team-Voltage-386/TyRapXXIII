//A field tag defines a tag, using its id, within field space as an xyz coordinate. 
// x spans from red to blue (so we dont care here), y is the line drawn through the april tags. Z is height obviously.
package frc.robot.constantsHelpers;

public class FieldTag {
    public static double x,y,z;
    public static int id;
    
    public FieldTag(int cid,double cx, double cy, double cz){
        id=cid;
        x=cx;
        y=cy;
        z=cz;
    }

}