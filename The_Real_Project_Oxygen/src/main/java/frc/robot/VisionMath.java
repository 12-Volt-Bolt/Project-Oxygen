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

    private double d6;

    private double beta;

    private double alpha;

    private double Theta;

    private double forwardDis;

    private double lateralOffSet;

    private double visionTargerSpan;
    
    private double xCenterCoordiante;


    public void VisionMath(double cameraCenterOffSet, double xCenterCoordiante, double distanceToCalPlusCalToVT, double visionTargerSpan) {
        d1 = cameraCenterOffSet;
        d2 = distanceToCalPlusCalToVT;
        this.xCenterCoordiante = xCenterCoordiante;
        this.visionTargerSpan = visionTargerSpan;
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
       // d3 = (d2) / Math.cos(alpha); error, error, error
    }

    public void calculateD4() {
       d4 = d1 * Math.sin(alpha);
    }
    public void calculateD5() {
        d5 = d1 * Math.cos(alpha);
    }

    public void calculateD6() {
        d6 = d2 * (d6 / d2);
    }

    public void calcLateralOffSet() {
      lateralOffSet = d4;
    }

    public void calcForwardDis() {
       forwardDis = d5;
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
    public double getAlphaRotation() {
        return alpha;
    }
    public double getBeta() {
        return beta;
    }

    public double getLateralOffSet() {
        return  lateralOffSet;
    }

    public double getForwardDis() {
       return  forwardDis;
    }
}
