package org.carlmontrobotics;

import java.security.spec.EncodedKeySpec;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;

public final class Constants {
    // public static final class Drivetrain {
    //     public static final double MAX_SPEED_MPS = 2;
    // }



    public static final double[] kP = {/*/Top/*/0.0, /*/Bottom/*/0.0, /*/Pincher/*/0.0, /*/Arm/*/0.0018};//0.0018
    public static final double[] kI = {/*/Top/*/0.0, /*/Bottom/*/0.0, /*/Pincher/*/0.0, /*/Arm/*/0.0};//DO NOT USE
    public static final double[] kD = {/*/Top/*/0.0, /*/Bottom/*/0.0, /*/Pincher/*/0.0, /*/Arm/*/0.08};//0.08

    public static final double[] kS = {/*/Top/*/0.0, /*/Bottom/*/0.0, /*/Pincher/*/0.0, /*/Arm/*/0.0};//DOES NOT WORK
    public static final double[] kV = {/*/Top/*/0.0, /*/Bottom/*/0.0, /*/Pincher/*/0.0, /*/Arm/*/0.0};//NOT IMPLEMENTED
    public static final double[] kA = {/*/Top/*/0.0, /*/Bottom/*/0.0, /*/Pincher/*/0.0, /*/Arm/*/0.0};//NOT IMPLEMENTED
    public static final double[] kG = {/*/Top/*/0.0, /*/Bottom/*/0.0, /*/Pincher/*/0.0, /*/Arm/*/0.63};//0.63
    
    public static final class OI {
        public static final double MIN_AXIS_TRIGGER_VALUE = 0.2;

        public static final class Driver {
            public static final int driverPort = 0;
            public static final int EFFECTOR_TOP_MOTOR_ID = 31;
            public static final int EFFECTOR_BOTTOM_MOTOR_ID = 33;
            public static final int EFFECTOR_PINCHER_MOTOR_ID = 31;
            public static final int EFFECTOR_ARM_MOTOR_ID = 34;
            public static final int effectorDistanceSensorID = 5;
			/*public static final int A = 1;
	    	public static final int B = 2;
	 		public static final int X = 3;
	    	public static final int Y = 4;*/
            //Not neccesary
        	public static final int leftBumper = 5;
        	public static final int rightBumper = 6;
        }
        public static final class Manipulator {
            public static final int manipulatorPort =  2;
            //public static final int X = 0;
            public static final Axis OuttakeTrigger = Axis.kRightTrigger;
            public static final Axis IntakeTrigger = Axis.kLeftTrigger;
            public static final int OuttakeBumper = Button.kRightBumper.value;
            public static final int INTAKE_BUMPER = Button.kLeftBumper.value;
        }
    }
	public static final class AlgaeEffectorc {

        //EFFECTOR

        
		// public static final int UPPER_MOTOR_PORT = 1; 
		// public static final int LOWER_MOTOR_PORT = 2;
        public static final int PINCH_MOTOR_PORT = 3;
        public static final int ARM_MOTOR_PORT = 20;
        public static final int aChannelEnc = 0;
        public static final int bChannelEnc = 1;

		public static final int TOP_ARRAY_ORDER = 0;
		public static final int BOTTOM_ARRAY_ORDER = 1;
		public static final int PINCHER_ARRAY_ORDER = 2;
        public static final int ARM_ARRAY_ORDER = 3;
        //the ArrayOrder variables replaced the ones for the kS since they just indicate the order and are the same for all PID values
        //TODO find these values out 
        public static double INTAKE_TOP_RPM = 1000;  
        public static double INTAKE_BOTTOM_RPM = 1000;  
        public static double INTAKE_PINCHER_RPM = 1000;  

        public static double OUTTAKE_TOP_RPM = -2100;  
        public static double OUTTAKE_BOTTOM_RPM = -2100;  
        public static double OUTTAKE_PINCHER_RPM = -2100;  

        public static double SHOOT_TOP_RPM = -2100;//ask design
        public static double SHOOT_BOTTOM_RPM = -2100; 
        public static double SHOOT_PINCHER_RPM = -2100; 

        public static double DEALGAFY_TOP_RPM = 1000;  
        public static double DEALGAFY_BOTTOM_RPM = 1000;  
        public static double DEALGAFY_PINCHER_RPM = 1000; 

        public static double RPM_ALLOWED_ERROR = 150;//rpm

        public static final int TBE_CPR = 8192; //Through-Bore Encoder
        public static final double TBE_DPP = 360.0/TBE_CPR; //Degrees per pulse
        public static final boolean invertedTBE = false; //if the encoder needs to read invertedly
        public static final CounterBase.EncodingType encodingType = Encoder.EncodingType.k2X;
        
        public static final double ARM_CHAIN_GEARING = 1;// TODO: set to 16.0/34
        public static final double ARM_GEAR_RATIO = 1.0/3;
        //TODO figure the zero out once encoder is on
        public static final double ARM_ZERO_ROT = Units.degreesToRotations(70);
        //TODO ask samo for angle to intake algae from pure vertical down
        public static final double ARM_INTAKE_ANGLE = 0;
        //TODO Figure these two out if we will be shooting algae
        public static final double ARM_RAMP_UP_ANGLE = 0;
        public static final double ARM_SHOOT_ANGLE = 0;
        //TODO Figure angle for dealgafying
        public static final double ARM_DEALGAFYING_ANGLE = 0;
        //TODO figure out resting angle of the arm while algae inside
        public static final double ARM_RESTING_ANGLE_WHILE_INTAKE_ALGAE = 0.0;
        //TODO figure out current threshold for pincher wheels
        public static final double PINCHER_CURRENT_THRESHOLD = 15.0;

        public static final double UPPER_ANGLE_LIMIT = -20;
        public static final double LOWER_ANGLE_LIMIT = -93;
        public static final double ROTATION_TO_DEG = 360;
        public static final double DEGREES_TO_RADS = Math.PI/180;
        public static final double ARM_DISCONT_DEG = -35;
        public static TrapezoidProfile.Constraints TRAP_CONSTRAINTS;
        public static final double MAX_FF_VEL_RAD_P_S = (Math.PI * .5)/2;
		public static final double MAX_FF_ACCEL_RAD_P_S = (53.728 / 4)/2;
        public static final double ARM_ERROR_MARGIN = 5;

        public static final double ARM_SYS_ID_START_COMMAND_ANGLE = -22; //TODO:


	}
    
}
