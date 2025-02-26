package org.carlmontrobotics;

import edu.wpi.first.math.util.Units;

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

    public static final class Limelightc {
        public static final String REEF_LL = "limelight-reef";

        public static final int[] REEF_IDS = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};

        // throw on drivetrain once i copy paste
        public static final double STD_DEV_X_METERS = 0.7; // uncertainty of 0.7 meters on the field
		public static final double STD_DEV_Y_METERS = 0.7; // uncertainty of 0.7 meters on the field
		public static final int STD_DEV_HEADING_RADS = 9999999; // (gyro) heading standard deviation, set extremely high
        public static final int MAX_TRUSTED_ANG_VEL_DEG_PER_SEC = 720; // maximum trusted angular velocity

        public static final double HEIGHT_FROM_GROUND_METERS_REEF = Units.inchesToMeters(0); // height off the ground
        public static final double MOUNT_ANGLE_DEG_INTAKE = -29; // angle (left/right)


    }
}
