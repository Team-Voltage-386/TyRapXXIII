// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import frc.robot.constantsHelpers.FieldTag;
import frc.robot.constantsHelpers.Grid;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class PhotonVisionConstants{
    public static int indexAprilTagPipeline=0;
    public static int indexRetroreflectivePipeline=1;
    public static double mountHeight=0.95;
    public static double mountAngle=41;
    public static double normytagdepth=14.53125/39.37;
    public static double normyTagHeight=18.125/39.37;
    public static double normyPoleWidthOff=(9.25+6.25+3+3+0.46875)/39.37;
    public static double tallPoleDepth=39.75/39.37;
    public static double tallPoleHeight=43.875/39.37;
    public static double shortPoleDepth=22.75/39.37;
    public static double shortPoleHeight=24.125/39.37;//inches to meters conversions (divide inches by 39.37)
    public static double AA=1.0; //the maximum error we will allow to differentiate between rows
    public static Grid g1= new Grid(1, normytagdepth, 0, normyTagHeight, true,tallPoleDepth, shortPoleDepth, normyPoleWidthOff, tallPoleHeight, shortPoleHeight,false,180);
    public static Grid[] grids = {g1};
    //FRC coordinate system. Origin is somewhere in the blue (literally blue alliance human player side) (inches)
    public static FieldTag at1=new FieldTag(1, 610.77/39.37, 42.19/39.37, 18.22/39.37,true,180);//xyz with inches to meters conversions
    public static FieldTag at2=new FieldTag(2, 610.77/39.37, 108.19/39.37, 18.22/39.37,true,180);
    public static FieldTag at3=new FieldTag(3, 610.77/39.37, 174.19/39.37, 18.22/39.37,true,180);
    public static FieldTag at4=new FieldTag(4, 636.96/39.37, 265.74/39.37, 27.38/39.37,false,180);
    public static FieldTag at5=new FieldTag(5, 14.25/39.37, 265.74/39.37, 27.38/39.37,true,0);
    public static FieldTag at6=new FieldTag(6, 40.45/39.37, 174.19/39.37, 18.22/39.37,true,0);
    public static FieldTag at7=new FieldTag(7, 40.45/39.37, 108.19/39.37, 18.22/39.37,true,0);
    public static FieldTag at8=new FieldTag(8, 40.45/39.37, 42.19/39.37, 18.22/39.37,false,0);
    public static FieldTag[] tags={at1,at2,at3,at4,at5,at6,at7,at8};


    //run of the mill distance algorithm
    public static double distAlg(FieldTag which, double pitch){return (which.z-mountHeight)/Math.tan(Math.PI*(pitch+mountAngle)/180);}

  }
}
