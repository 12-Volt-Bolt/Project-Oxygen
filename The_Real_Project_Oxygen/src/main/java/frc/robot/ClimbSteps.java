/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.subsystems.GenericLiftSubsystem.LiftID;
import frc.robot.statics_and_classes.DataParser;

/**
 * Add your docs here.
 */
public class ClimbSteps {

    public static final String climbStepData = "";

    public static ClimbStep[] CLIMB_STEPS;

    public static void ParseSteps(String inputData) {
        CLIMB_STEPS = DataParser.ParseObject(inputData, "ClimbDataInput[]");
    }

    public static ClimbStep GetStep(int step) {
        return CLIMB_STEPS[step];
    }

    public static ClimbSubstep GetSubstep(int substep, ClimbStep step) {
        return step.substeps[substep];
    }

    public static class ClimbStep {
        public ClimbSubstep[] substeps;

        public ClimbStep(ClimbSubstep[] newSubsteps){
            substeps = newSubsteps;
        }
    }
    
    public static class ClimbSubstep {
        public LiftID liftID;
        public Boolean lockPos;
        public double speed;
        public double distance;

        public ClimbSubstep(LiftID newLiftID, Boolean newLockPos, double newSpeed, double newDistance) {
            liftID = newLiftID;
            lockPos = newLockPos;
            speed = newSpeed;
            distance = newDistance;
        }

//        public static ClimbSubstep ParseAndAssign(String stringInput) {
//            ClimbSubstep temp = Parse(stringInput);
//            liftID = temp.liftID;
//            lockPos = temp.lockPos;
//            speed = temp.speed;
//            distance = temp.distance;
//
//            return temp;
//        }
    }

    public static class ClimbDataInput {
        public ClimbStep[] climbSteps;

        public ClimbDataInput(ClimbStep[] newClimbSteps) {
            climbSteps = newClimbSteps;
        }
    }


}

