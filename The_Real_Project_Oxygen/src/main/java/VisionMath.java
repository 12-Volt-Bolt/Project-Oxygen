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
    
    // Alignment line angle
    private double Alpha;
    
    private double alphaRotate;

    // Distance to Cal Plus Cal to VT
    private double d1;

    // camera center offset
    private double d2;

    //Please see method
    private double d3;
    
    private double d4;

    private double d5;

    private double beta;

    private double alpha;

    private double forWardDis;
    


    public void VisionMath(int cameraCenterOffSet, int distanceToCalPlusCalToVT) {
        d1 = cameraCenterOffSet;
        d2 = distanceToCalPlusCalToVT;
    }


    /* Do we really need this?
    public void calculateD1() {
       
    }
   */
  /* Do we really need this?
    public void calculateD2() {
        
    }
    */
    public void calculateD3() {
        d3 = (d2) / Math.cos(alpha);
    }
    public void calculateD4() {
        
    }
    public void calculateD5() {
        
    }

    public void calcLateralOffSet() {

    }

    public void calcForwardDis() {

    }

    public void calcAlpha() {
        Alpha = beta - 90;
    }

    public double getD1() {
       return d1;
    }
    public double getD2() {
        return d2;

    }
    public double getD3() {
        return d3;

    }
    public double getD4() {
        return d4;

    }
    public double getD5() {
        return d5;

    }
    public double getAlpha() {
        return alpha;
    }
    public double getBeta() {
        return beta;
    }
}
