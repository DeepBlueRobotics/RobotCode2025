package org.carlmontrobotics;

public final class Constants {
    // public static final class Drivetrain {
    //     public static final double MAX_SPEED_MPS = 2;
    // }
    public static final class OI {
        public static final class Driver {
            public static final int port = 0;
			public static final int A = 1;
	    	public static final int B = 2;
	 		public static final int X = 3;
	    	public static final int Y = 4;
        	public static final int leftBumper = 5;
        	public static final int rightBumper = 6;
            public static final double MIN_AXIS_TRIGGER_VALUE = 0.2;
        }
        public static final class Manipulator {
            public static final int port = 1;
        }
        public static final int X = 0;
    }
	public static final class AlgaeEffectorc {
		public static final int upperMotorID = 1;
		public static final int lowerMotorID = 2;
	}
}
