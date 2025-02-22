package org.carlmontrobotics;

import java.security.spec.EncodedKeySpec;

import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;

public final class Constants {
    // public static final class Drivetrain {
    //     public static final double MAX_SPEED_MPS = 2;
    // }



    public static final double[] kP = {/*/Top/*/0.0, /*/Bottom/*/0.0, /*/Pincher/*/0.0, /*/Arm/*/0.0};
    public static final double[] kI = {/*/Top/*/0.0, /*/Bottom/*/0.0, /*/Pincher/*/0.0, /*/Arm/*/0.0};
    public static final double[] kD = {/*/Top/*/0.0, /*/Bottom/*/0.0, /*/Pincher/*/0.0, /*/Arm/*/0.0};

    public static final double[] kS = {/*/Top/*/0.0, /*/Bottom/*/0.0, /*/Pincher/*/0.0, /*/Arm/*/0.0};
    public static final double[] kV = {/*/Top/*/0.0, /*/Bottom/*/0.0, /*/Pincher/*/0.0, /*/Arm/*/0.0};
    public static final double[] kA = {/*/Top/*/0.0, /*/Bottom/*/0.0, /*/Pincher/*/0.0, /*/Arm/*/0.0};

    public static final class OI {
        public static final double MIN_AXIS_TRIGGER_VALUE = 0.2;

        public static final class Driver {
            public static final int driverPort = 0;
            public static final int effectorMotorID = 4;
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
            public static final int IntakeBumper = Button.kLeftBumper.value;
        }
    }
	public static final class AlgaeEffectorc {
		public static final int upperMotorID = 1;
		public static final int lowerMotorID = 2;
        public static final int pinchMotorID = 3;
        public static final int armMotorID = 4;
        public static final int aChannelEnc = 0;
        public static final int bChannelEnc = 1;

		public static final int topArrayOrder = 0;
		public static final int bottomArrayOrder = 1;
		public static final int pincherArrayOrder = 2;
        public static final int armArrayOrder = 3;
        //the ArrayOrder variables replaced the ones for the kS since they just indicate the order and are the same for all PID values
        //TODO find these values out vvv
        public static double intakeTopRPM = 1000;  
        public static double intakeBottomRPM = 1000;  
        public static double intakePincherRPM = 1000;  

        public static double outtakeTopRPM = 2100;  
        public static double outtakeBottomRPM = 2100;  
        public static double outtakePincherRPM = 2100;  

        public static final double TBE_PPR = 2048.0; //Through-Bore Encoder
        public static final double TBE_DPP = 360.0/TBE_PPR; //Degrees per pulse
        public static final boolean invertedTBE = false; //if the encoder needs to read invertedly
        public static final CounterBase.EncodingType encodingType = Encoder.EncodingType.k2X;

        public static final double armGearing = 39.375;

        //TODO figure the zero out once encoder is on
        public static final double armToZero = 0; //Pure vertical down
        //TODO ask samo for angle to intake algae from pure vertical down
        public static final double armIntakeAngle = 0;
        //TODO Figure these two out if we will be shooting algae
        public static final double armRampingUpAngle = 0;
        public static final double armShootingAngle = 0;
        //TODO Figure angle for dealgafying
        public static final double armDealgafyngAngle = 0;
        //TODO figure out resting angle of the arm while algae inside
        public static final double armRestingAngleWhileIntakeAlgae = 0;

        //max angle in radians and minimum angle in radians
        public static final double UPPER_ANGLE_LIMIT_RAD = 2.6;
		public static final double LOWER_ANGLE_LIMIT_RAD = 0;
        public static final double ARM_DISCONT_RAD = (LOWER_ANGLE_LIMIT_RAD + UPPER_ANGLE_LIMIT_RAD) / 2 - Math.PI;

	}
}
