package org.carlmontrobotics;

public final class Constants {
    // public static final class Drivetrain {
    //     public static final double MAX_SPEED_MPS = 2;
    // }
    public static final class OI {
        public static final class Driver {
            public static final int port = 0;
        }
        public static final class Manipulator {
            public static final int port = 1;
        }
    }
    public static final class CoralEffectorConstants{
        public final static int coralMotorPort = 1;
        public final static int coralLimitSwitchPort = 0;
        public final static int coralDistanceSensorPort = 5;
    }
}
