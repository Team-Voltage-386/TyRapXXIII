package frc.robot.constantsHelpers;

public class RRTarget {
    public static double x,y,z; //RELATIVE TO THE APRILTAG PARENT
    public boolean available;
    public int q;
     
    public RRTarget(double cx, double cy, double cz, int cq){
        x=cx;
        y=cy;
        z=cz;
        available=true;
        q=cq;
    }

    public void setAvailable(boolean i){
        available=i;
    }
}
