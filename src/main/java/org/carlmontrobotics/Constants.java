package org.carlmontrobotics;

import edu.wpi.first.wpilibj.StadiaController.Button;

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
            public static final int IntakeButton = Button.kA.value;
        }
    }
    public static final class CoralEffectorConstants{
        public final static int coralMotorPort = 1;
        public final static int coralLimitSwitchPort = 0;
        public final static int coralDistanceSensorPort = 6;
        public final static int coralDistanceSensorDistance = 150;
        public final static double coralEffectorMotorInputFastSpeed = 0.07;
        // public final static double coralEffectorMotorInputFastSpeed2 = 0.05;
        // public final static double coralEffectorMotorInputSlowSpeed = 0.04;
        public final static double coralEffectorMotorOutputSpeed = 0.2;
        public final static double coralEffectorDistanceSensorOffset = -0.1;
        public final static double kp = 0.04;
        public final static double ki = 0.0;
        public final static double kd = 0.0;
    }
}
