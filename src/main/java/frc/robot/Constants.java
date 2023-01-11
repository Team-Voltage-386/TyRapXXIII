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
    public static double mountHeight=0.0;
    public static double mountAngle=0.0;
    public static double normytagdepth=14.53125*.0254;
    public static double normyTagHeight=18.125*.0254;
    public static double normyPoleWidthOff=(9.25+6.25+3+3+0.46875)*.0254;
    public static double tallPoleDepth=39.75*.0254;
    public static double tallPoleHeight=43.875*.0254;
    public static double shortPoleDepth=22.75*.0254;
    public static double shortPoleHeight=24.125*.0254;
    public static double AA=1.0; //the maximum error we will allow to differentiate between rows
    public static Grid g1= new Grid(1, normytagdepth, 0, normyTagHeight, tallPoleDepth, shortPoleDepth, normyPoleWidthOff, tallPoleHeight, shortPoleHeight,false);
    public static Grid[] grids = {g1};
    //run of the mill distance algorithm
    public static double distAlg(FieldTag which, double pitch){return (which.z-mountHeight)/Math.tan(Math.PI*(pitch+mountAngle)/180);}

  }
}
