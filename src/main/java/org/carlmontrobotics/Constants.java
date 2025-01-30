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
        //Positions
        public static final double downPos = 0;
        public static final double l1 = 0;
        public static final double l2 = 0;
        public static final double l3 = 0;
        public static final double l4 =0;
        public static final double net = 0;
        public static final double processor = 0;
        //ScoreENUM
        public enum ElevatorPos {
            DOWN(downPos),
            L1(l1),
            L2(l2),
            L3(l3),
            L4(l4),
            NET(net),
            PROCESSOR(processor);

            public final double positionInches;
            ElevatorPos(double positionInches) {
                this.positionInches = positionInches;
            }
        }
        //Tolerance
        public static final double elevatorTolerance = 0.4;
        
    }
}
