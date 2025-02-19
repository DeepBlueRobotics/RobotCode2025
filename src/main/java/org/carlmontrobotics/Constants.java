package org.carlmontrobotics;

public final class Constants {
    // public static final class Drivetrain {
    //     public static final double MAX_SPEED_MPS = 2;
    // }
    public static final class Limelightc {
        public static final String ROBOT_LL_NAME = "limelight-shooter"; // Please change the constant

        public static final int[] VALID_IDS = {3, 16}; // Processors
        public static final double MOUNT_ANGLE_DEG_LL = 45; // Please change
        public static final double HEIGHT_FROM_GROUND_METERS_ROBOT = 0.206502; // Seems to be OK
        public static final double ALGAE_HEIGHT = 0.4191;

        public static final double STD_DEV_X_METERS = 0.7; // uncertainty of 0.7 meters on the field
		public static final double STD_DEV_Y_METERS = 0.7; // uncertainty of 0.7 meters on the field
		public static final int STD_DEV_HEADING_RADS = 9999999; // (gyro) heading standard deviation, set extremely high

        public static final class Apriltag {
			/*public static final int RED_SPEAKER_CENTER_TAG_ID = 4;
			public static final int BLUE_SPEAKER_CENTER_TAG_ID = 7;
			public static final double SPEAKER_CENTER_HEIGHT_METERS = Units.inchesToMeters(60.125);
			public static final double HEIGHT_FROM_BOTTOM_TO_SUBWOOFER = Units.inchesToMeters(26);
			public static final double HEIGHT_FROM_BOTTOM_TO_ARM_RESTING = Units.inchesToMeters(21.875);*/

            public static final int RED_PROCESSOR_ID = 3;
            public static final int BLUE_PROCESSOR_ID = 16;
            public static final double PROCESSOR_CENTER_HEIGHT_METERS = 0.4318; // 17 inches. 17 wasted seconds of my time.
		}
    }

    public static final class Drivetrainc {
        // Drivetrain BS that can be added later
    }

    public static final class OI {
        public static final class Driver {
            public static final int port = 0;
        }
        public static final class Manipulator {
            public static final int port = 1;

        }
    }
}
