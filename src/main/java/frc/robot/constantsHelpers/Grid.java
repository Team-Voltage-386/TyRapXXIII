package frc.robot.constantsHelpers;

public class Grid {
    public FieldTag gutenTag;//gTag
    public RRTarget[] ticTags=new RRTarget[4];//tictactags
    public boolean community;

    public Grid(int cAID,double cAX,double cAY,double cAZ,double cPXt,double cPXs,double cPY,double cPZt,double cPZs,boolean com){
        community=com;
        gutenTag=new FieldTag(cAID, cAX, cAY, cAZ);
        for(int i=0; i<4;i++){
            double q=Math.atan((i*Math.PI/2)+Math.PI/4);
            double x=0.0; double y=cPY; double z=0.0;
            if(Math.sin(q)>0){
                x=cPXt;
                z=cPZt;
            } else {
                x=cPXs;
                z=cPZs;
            }
            if(Math.cos(q)<0){
                y*=-1;
            }
            ticTags[i]=new RRTarget(x,y,z,i+1);
        }
    }

    public FieldTag getTag(){
        return this.gutenTag;
    }
    public RRTarget getRR(int q){
        return this.ticTags[q-1];
    }
    public RRTarget[] getRR(){
        return this.ticTags;
    }


}
