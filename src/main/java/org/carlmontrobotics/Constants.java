package org.carlmontrobotics;

import edu.wpi.first.wpilibj.XboxController.Axis;

public final class Constants {
    // public static final class Drivetrain {
    //     public static final double MAX_SPEED_MPS = 2;
    // }

    public static final int top = 1;
    public static final int bottom = 1;
    public static final int pincher = 1;

    public static final double[] kP = {/*/Top/*/0.0, /*/Bottom/*/0.0, /*/Pincher/*/0.0};
    public static final double[] kI = {/*/Top/*/0.0, /*/Bottom/*/0.0, /*/Pincher/*/0.0};
    public static final double[] kD = {/*/Top/*/0.0, /*/Bottom/*/0.0, /*/Pincher/*/0.0};

    public static final double[] kS = {/*/Top/*/0.0, /*/Bottom/*/0.0, /*/Pincher/*/0.0};
    public static final double[] kV = {/*/Top/*/0.0, /*/Bottom/*/0.0, /*/Pincher/*/0.0};
    public static final double[] kA = {/*/Top/*/0.0, /*/Bottom/*/0.0, /*/Pincher/*/0.0};

    public static final class OI {
        public static final double MIN_AXIS_TRIGGER_VALUE = 0.2;

        public static final class Driver {
            public static final int driverPort = 0;
			/*public static final int A = 1;
	    	public static final int B = 2;
	 		public static final int X = 3;
	    	public static final int Y = 4;*/
            //Not neccesary
        	public static final int leftBumper = 5;
        	public static final int rightBumper = 6;
        }
        public static final class Manipulator {
            public static final int manipulatorPort =  5;
            //public static final int X = 0;
            public static final Axis OuttakeTrigger = Axis.kRightTrigger;
            public static final Axis IntakeTrigger = Axis.kLeftTrigger;
        }
    }
	public static final class AlgaeEffectorc {
		public static final int upperMotorID = 1;
		public static final int lowerMotorID = 2;
	}
    public static final class CoralEffectorc {
		public static final int effectorMotorID = 3;
        public static final int effectorDistanceSensorID = 4;
        public static final double kP = 0.0;
		public static final double kI = 0.0;
		public static final double kD = 0.0;
        public static double intakeRPM = 1000;  
        public static double outtakeRPM = 2100;  
        //TODO: Change after testing
        public static final int DETECT_DISTANCE_INCHES = 3;
	}
}
