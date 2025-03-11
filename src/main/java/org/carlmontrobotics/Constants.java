package org.carlmontrobotics;

import edu.wpi.first.wpilibj.StadiaController.Button;

public final class Constants {
    public static final class OI {
        public static final class Driver {
            public static final int DRIVE_CONTROLLER_PORT = 0;
        }
        public static final class Manipulator {
            public static final int MANIPULATOR_CONTROLLER_PORT = 1;
            public static final int OUTTAKE_BUTTON = Button.kA.value;
            public static final int INTAKE_BUTTON = Button.kB.value;
        }
    }
    public static final class CoralEffectorc{
        public final static int CORAL_MOTOR_PORT = 30;
        public final static int CORAL_LIMIT_SWITCH_PORT = 0;
        public final static int CORAL_DISTANCE_SENSOR_PORT = 6;
        public final static int CORAL_DISTANCE_SENSOR_DISTANCE = 150; //mm
        public final static double CORAL_INTAKE_ERR = .1;//encoder units - rotations
        public final static double INPUT_FAST_SPEED = 0.07; //TODO: tune this
        public final static double INPUT_SLOW_SPEED = 0.035; //TODO: tune this
        public final static double OUTPUT_SPEED = 0.1; //TODO: tune this
        public final static double CORAL_EFFECTOR_DISTANCE_SENSOR_OFFSET = -0.1; //TOD: tune this?
        public final static double KP = 1.3; //TODO: tune this
        public final static double KI = 0;
        public final static double KD = 0;
        public final static double INTAKE_TIME_OUT = 0.5;
        public final static double OUTTAKE_TIME_OUT = 10;
        public final static double MANUAL_INTAKE_TIME_OUT = 1;

        public final static int LIMIT_SWITCH_PORT = 7; //TODO: Change
    }
}
