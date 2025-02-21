package org.carlmontrobotics;

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

		public static final int TopArrayOrder = 0;
		public static final int BottomArrayOrder = 1;
		public static final int PincherArrayOrder = 2;
        public static final int ArmArrayOrder = 3;
        //the ArrayOrder variables replaced the ones for the kS since they just indicate the order and are the same for all PID values

        public static double intakeTopRPM = 1000;  
        public static double intakeBottomRPM = 1000;  
        public static double intakePincherRPM = 1000;  

        public static double outtakeTopRPM = 2100;  
        public static double outtakeBottomRPM = 2100;  
        public static double outtakePincherRPM = 2100;  

	}
}
