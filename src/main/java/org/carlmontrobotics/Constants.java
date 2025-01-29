package org.carlmontrobotics;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

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
    public static final class Elevatorc {
        //ports
        public static final int masterPort = 19;
        public static final int followerPort = 20;

        //Config
        public static final IdleMode masterIdleMode = IdleMode.kBrake;
        public static final IdleMode followerIdleMode = IdleMode.kBrake;
        public static final boolean masterInverted = true;
        public static final boolean followerInverted = true;
        public static final double masterPositionConversionFactor = 1000;
        public static final double followerPositionConversionFactor = 1000;
        public static final double masterVelocityConversionFactor = 1000;
        public static final double followerVelocityConversionFactor = 1000;

        //PID
        public static final double kP = 1;
        public static final double kI = 0;
        public static final double kD = 0;
        
    }
}
