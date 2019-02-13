/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * This class handles all vision processing calculations.
 * This class will constantly be updated with variables during the teleop and auto periods
 * Please see the vision processing command to see class utilization
 */
public class VisionMath {
    
    private double beta;
    private double theta;

    // The angle of the alignment line relative to the robot
    // This variable is also equaled to rotation
    // 90 - Beta = Alpha
    private double alpha;
    
    // d1 is equaled to the robot's distance away from the vision targets
    private double linearDistanceAwayFromVisionTarget;

    // This distance is equaled to d2
    // d2 is the Camera's center offset
    private double cameraCenterOffset;
    
    //private double d3 is no longer used

    // This variable is the lateral  offset robot from the center of the vision target 
    // This distance is equaled to d4
    private double lateralOffSet;

    // This variable is equaled to d5
    // The robot must move a net distance of d5 in order to reach the cargo station for disk placement
    private double forwardDistance;
    
    // The following data will be received from the dashboard and are calculated by the LabVIEW vision program
    private int MeasCenterXInPixels = 0;
    private int MeasTargetSeparationInPixels = 0;
    private double MeasAlignmentLineAngleInDegrees = 0;

    //Calibration Data
    // TODO: these values will be updated when the competition bot is built and ready to be utlized for testing
    public static final int calCenterXInPixels = 0;
    public static final int calcDistanceInCm = 0;
    public static final int calTargetSeparationInPixels = 0;
    public static final int calAlignmentLineAngleInDegrees = 0;
    public static final double calLateralOffsetConstant = 20.32;
    


    public void VisionMath(int  centerXInPixels, int targetSeparationInPixels, double alignmentLineAngleInDegrees) {
        this.MeasCenterXInPixels = centerXInPixels;
        this.MeasTargetSeparationInPixels = targetSeparationInPixels;
        this.MeasAlignmentLineAngleInDegrees = alignmentLineAngleInDegrees;
    }

    public void calcDistanceFromCamToTarget() {
        linearDistanceAwayFromVisionTarget = (calTargetSeparationInPixels / MeasTargetSeparationInPixels) * calcDistanceInCm;
    }

    public void calcLateralOffSet() {
        lateralOffSet = (calCenterXInPixels - MeasCenterXInPixels) - calcDistanceInCm;
      }

      public void calcAlpha() {
        alpha = MeasAlignmentLineAngleInDegrees;
    }

    public void calcForwardDistanceCm() {
        forwardDistance = linearDistanceAwayFromVisionTarget * (Math.cos(alpha));
    }

    // alpha
    public double getAlphaRotation() {
        return alpha;
    }
    // beta
    public double getBeta() {
        return beta;
    }
    

    // theta
    public double returnTheta() {
        return theta;
    }

    // d1
    public double returnDistanceFromCamToTarget() {
        return linearDistanceAwayFromVisionTarget;
    }
    
    // d2
    public double getCameraCenterOffset() {
        return cameraCenterOffset;
    }

    // d4
    public double getLateralOffSet() {
        return  lateralOffSet;
    }
    // d5
    public double getForwardDistance() {
       return   forwardDistance;
    }

    //d6 
    public double getAngleOffSetInDegrees() {
        return MeasAlignmentLineAngleInDegrees - calAlignmentLineAngleInDegrees;
    }
}
