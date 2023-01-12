package frc.robot.constantsHelpers;

//i will need to rename these things to be logical. Anyways, organizationally it makes sense to define things by the grid. For just april tag features, perhaps it is a bit excessive. I will likely delete these things
public class Grid {
    public FieldTag fieldTag;//gutenTag
    public RRTarget[] rrTargets=new RRTarget[4];//tictactags
    public boolean community;

    public Grid(int ID,double TagX,double TagY,double TagZ,boolean IsGrid,double TallPoleRelativeX,double ShortPoleRelativeX,double TallPoleRelativeY,double TallPoleRelativeZ,double ShortPoleRelativeZ,boolean Community,double orientation){
        community=Community;
        fieldTag=new FieldTag(ID, TagX, TagY, TagZ,IsGrid,orientation);
        for(int i=0; i<4;i++){
            double q=Math.atan((i*Math.PI/2)+Math.PI/4);
            double x=0.0; double y=TallPoleRelativeY; double z=0.0;
            if(Math.sin(q)>0){
                x=TallPoleRelativeX;
                z=TallPoleRelativeZ;
            } else {
                x=ShortPoleRelativeX;
                z=ShortPoleRelativeZ;
            }
            if(Math.cos(q)<0){
                y*=-1;
            }
            rrTargets[i]=new RRTarget(x,y,z,i+1);
        }
    }
    public Grid(FieldTag Tag, double cPXt,double cPXs,double cPY,double cPZt,double cPZs,boolean com){
        community=com;
        fieldTag=Tag;
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
            rrTargets[i]=new RRTarget(x,y,z,i+1);
        }
    }

    public FieldTag getTag(){
        return this.fieldTag;
    }
    public RRTarget getRR(int q){
        return this.rrTargets[q-1];
    }
    public RRTarget[] getRR(){
        return this.rrTargets;
    }


}
