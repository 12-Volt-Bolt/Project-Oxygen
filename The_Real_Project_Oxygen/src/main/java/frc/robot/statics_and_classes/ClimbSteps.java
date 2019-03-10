/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.statics_and_classes;

import frc.robot.subsystems.GenericLiftSubsystem.LiftID;
import frc.robot.statics_and_classes.DataParser;

/**
 * Add your docs here.
 */
public class ClimbSteps {

    private static DataParser parser = new DataParser();

    public static final String LV_3_CLIBM_DATA = "testLift, false, 0.5, 50";

    public static final ClimbStep[] LV_3_CLIMB_STEPS = parser.ParseObject(LV_3_CLIBM_DATA, "ClimbDataInput[]");

    public static ClimbStep[] ParseSteps(String inputData) {
        //LV_3_CLIMB_STEPS = DataParser.ParseObject(inputData, "ClimbDataInput[]");
        return parser.ParseObject(LV_3_CLIBM_DATA, "ClimbStep[]");
    }

    public static ClimbStep GetStep(int step) {
        return LV_3_CLIMB_STEPS[step];
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
        public int distance;

        public ClimbSubstep(LiftID newLiftID, Boolean newLockPos, double newSpeed, int newDistance) {
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

