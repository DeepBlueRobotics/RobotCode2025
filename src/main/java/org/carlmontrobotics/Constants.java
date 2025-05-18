
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics;

import edu.wpi.first.wpilibj.StadiaController.Button;

import edu.wpi.first.wpilibj.StadiaController.Button;

import org.carlmontrobotics.lib199.swerve.SwerveConfig;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import static org.carlmontrobotics.Config.CONFIG;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController.Axis;
//import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.util.Color;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static final double[] kP = {/*/Top/*/0.0, /*/Bottom/*/0.0, /*/Pincher/*/0.0, /*/Arm/*/0.0018};//0.0018
    public static final double[] kI = {/*/Top/*/0.0, /*/Bottom/*/0.0, /*/Pincher/*/0.0, /*/Arm/*/0.0};//DO NOT USE
    public static final double[] kD = {/*/Top/*/0.0, /*/Bottom/*/0.0, /*/Pincher/*/0.0, /*/Arm/*/0.08};//0.08

    public static final double[] kS = {/*/Top/*/0.0, /*/Bottom/*/0.0, /*/Pincher/*/0.0, /*/Arm/*/0.0};
    public static final double[] kV = {/*/Top/*/0.0, /*/Bottom/*/0.0, /*/Pincher/*/0.0, /*/Arm/*/0.0};//NOT IMPLEMENTED
    public static final double[] kA = {/*/Top/*/0.0, /*/Bottom/*/0.0, /*/Pincher/*/0.0, /*/Arm/*/0.0};//NOT IMPLEMENTED
    public static final double[] kG = {/*/Top/*/0.0, /*/Bottom/*/0.0, /*/Pincher/*/0.0, /*/Arm/*/0.58};
	
	public static final double g = 9.81; // meters per second squared
	
	public static final class Elevatorc {
		// ports
		public static final int masterPort = 20;
		public static final int followerPort = 21; // inverted
		//public static final int elevatorTopLimitSwitchPort = 1;
		public static final int elevatorBottomLimitSwitchPort = 0;
		public static final double GEAR_RATIO = 1.0/20; //TODO: CHANGE TO ACTUAL GEAR RATIO

		// Config
		// TODO figure these parts out
		public static final double MAX_ACCEL_RAD_P_S = 1;
		public static final IdleMode masterIdleMode = IdleMode.kBrake; //TODO: Change back to break
		public static final IdleMode followerIdleMode = IdleMode.kBrake;
		public static final boolean masterInverted = false; 
		public static final boolean followerInverted = true;
		public static final double masterPositionConversionFactor = Units.inchesToMeters(2*GEAR_RATIO*(Math.PI * 1.76)); // 2*(gear_ratio*(pi*sprocket_pitch_diameter)) aka 2*1/20*pi*1.76
		public static final double masterVelocityConversionFactor = Units.inchesToMeters(2*GEAR_RATIO*(Math.PI * 1.76)*1.0/60);
		public static final double maxElevatorHeightInches = 52.5;
		public static final double minElevatorHeightInches = 0;
		
        //PID
        public static final double kP = 20;//45.476;
        public static final double kI = 0;
        public static final double kD = 10.609/2;
		//Feedforward
        public static final double kS = 0; //.12666
		public static final double kG = 0.177;
		public static final double kV = 8.9921;
		public static final double kA = 1.4586;
        //Positions
        public static final double downPos = 0;
        public static final double l1 = 0;
        public static final double l2 = Units.inchesToMeters(6.5-1.236220); 
        public static final double l3 = Units.inchesToMeters(22.5-1.236220); 
        public static final double l4 = Units.inchesToMeters(52.64-1.236220);
        public static final double net = Units.inchesToMeters(53.2-1.236220);
        public static final double processor = 0;
        public static final double bottomAlgaeRemoval = Units.inchesToMeters(22.5-1.236220);
        public static final double uppperAlgaeRemoval = Units.inchesToMeters(38.35-1.236220);
        //ScoreENUM
        public enum ElevatorPos {
            DOWN(downPos),
            L1(l1),
            L2(l2),
            L3(l3),
            L4(l4),
            NET(net),
            PROCESSOR(processor),
            BOTTOMALGAE(bottomAlgaeRemoval),
            UPPERALGAE(uppperAlgaeRemoval);

			public final double positionInches;

			ElevatorPos(double positionInches) {
				this.positionInches = positionInches;
			}

			public double getPositioninMeters() {
				return Units.degreesToRadians(positionInches);
			}
		}

		public static final double elevatorTolerance = 0.4;


	}// Tolerance
	

	public static final class Drivetrainc {
		// #region Subsystem Constants
		public static final double wheelBase = 24.75; //CONFIG.isSwimShady() ? Units.inchesToMeters(19.75) : Units.inchesToMeters(16.75);
		public static final double trackWidth = 24.75;//CONFIG.isSwimShady() ? Units.inchesToMeters(28.75) : Units.inchesToMeters(23.75);
		// "swerveRadius" is the distance from the center of the robot to one of the
		// modules
		public static final double swerveRadius = Math.sqrt(Math.pow(wheelBase / 2, 2) + Math.pow(trackWidth / 2, 2));
		// The gearing reduction from the drive motor controller to the wheels
		// Gearing for the Swerve Modules is 6.75 : 1e
		public static final double driveGearing = 6.75;
		// Turn motor shaft to "module shaft"
		public static final double turnGearing = 150.0 / 7;

		public static final double driveModifier = 1;
		public static final double wheelDiameterMeters = Units.inchesToMeters(4.0) * 7.36 / 7.65 /*
																						* empirical
																						* correction
																						*/;
		public static final double mu = 1; /* 70/83.2; */ // coefficient of friction. less means less max acceleration.
		public static final double ROBOTMASS_KG = 135;// max is 135kg
		// moment of inertia, kg/mm
		// calculated by integral of mass * radius^2 for every point of the robot
		// easy way? just do total mass * radius^2
		public static final double MOI = ROBOTMASS_KG * swerveRadius * swerveRadius;

		public static final double NEOFreeSpeed = 5676 * (2 * Math.PI) / 60; // radians/s
		// Angular speed to translational speed --> v = omega * r / gearing
		public static final double maxSpeed = NEOFreeSpeed * (wheelDiameterMeters / 2.0) / driveGearing;
		public static final double maxForward = maxSpeed; // todo: use smart dashboard to figure this out
		public static final double maxStrafe = maxSpeed; // todo: use smart dashboard to figure this out
		// seconds it takes to go from 0 to 12 volts(aka MAX)
		public static final double secsPer12Volts = 0.1;

		// maxRCW is the angular velocity of the robot.
		// Calculated by looking at one of the motors and treating it as a point mass
		// moving around in a circle.
		// Tangential speed of this point mass is maxSpeed and the radius of the circle
		// is sqrt((wheelBase/2)^2 + (trackWidth/2)^2)
		// Angular velocity = Tangential speed / radius
		public static final double maxRCW = maxSpeed / swerveRadius;

		public static final boolean[] reversed = { false, false, false, false };
		// public static final boolean[] reversed = {true, true, true, true};
		// Determine correct turnZero constants (FL, FR, BL, BR)
		public static final double[] turnZeroDeg = RobotBase.isSimulation() ? new double[] {-90.0, -90.0, -90.0, -90.0 }
		: (CONFIG.isHammerHead() ? new double[] { 85.7812, 85.0782, -96.9433, -162.9492 }
			: new double[] { 17.2266, -96.8555, -95.8008, 85.166 });/* real values here */

		// kP, kI, and kD constants for turn motor controllers in the order of
		// front-left, front-right, back-left, back-right.
		// Determine correct turn PID constants
		public static final double[] turnkP = CONFIG.isHammerHead() ? new double[] {4.0087, 60.885, 60.946, 60.986/2 } : 
			new double[]{50,50,50,50};//{1.9085, /*0.21577*/0.1, /*0.12356*/0.05, 0.36431};//sysid for fr that didnt't work{0.099412, 0.13414, 3.6809, 3.6809} //{49, 23,33, 28};//{51.078, 25, 35.946, 30.986}; // {0.00374, 0.00374, 0.00374,
																		// 0.00374};
		public static final double[] turnkI = {0, 0, 0, 0};//{ 0, 0.1, 0, 0 };
		public static final double[] turnkD = CONFIG.isHammerHead() ? new double[] { 0/* dont edit */, 0.5, 0.42, 1 } :
			new double[]{0, 0, 0, 0};//{ 0.2/* dont edit */, 0.3, 0.5, 0.4}; // todo: use d
		// public static final double[] turnkS = {0.2, 0.2, 0.2, 0.2};
		public static final double[] turnkS = CONFIG.isHammerHead() ? new double[]{ 0.12507, 0.17026, 0.2, 0.23262 }:
			new double[]{0.21969, 0.11487, 0.18525, 0.24865};//sysid for fr that didnt't work{0.041796, 0.09111, 0.64804, 1.0873}//{ 0.13027, 0.17026, 0.2, 0.23262 };

		// V = kS + kV * v + kA * a
		// 12 = 0.2 + 0.00463 * v
		// v = (12 - 0.2) / 0.00463 = 2548.596 degrees/s
		public static final double[] turnkV = CONFIG.isHammerHead()? new double[] { 2.6172, 2.7597, 2.7445, 2.7698 }:
			new double[] {2.7073, 2.6208, 2.7026, 2.7639};//sysid for fr that didnt't work{2.6403, 2.6603, 2.6168, 2.5002} //{2.6532, 2.7597, 2.7445, 2.7698};
		public static final double[] turnkA = CONFIG.isHammerHead()? new double[] { 1.2097, 0.17924, 0.17924, 0.17924 }:
			new double[]{0.18069, 0.06593, 0.17439, 0.2571};//sysid for fr that didnt't work{0.33266, 0.25535, 0.17924, 0.17924} //{ 0.17924, 0.17924, 0.17924, 0.17924 };

		// kP is an average of the forward and backward kP values
		// Forward: 1.72, 1.71, 1.92, 1.94
		// Backward: 1.92, 1.92, 2.11, 1.89
		// Order of modules: (FL, FR, BL, BR)
		public static final double[] drivekP = CONFIG.isHammerHead() ? new double[] { 1.75*1.275, 1.75*1.275, 1.75*1.275, 1.75*1.275 }
		: new double[] {1.99+.6, 1.99+.5, 1.99+ .1 , 1.99}; //{2.2319, 2.2462, 2.4136, 3.6862}; // {1.82/100, 1.815/100, 2.015/100,
																// 1.915/100};
		public static final double[] drivekI = CONFIG.isHammerHead()? new double[] {0,0,0,0} :
			new double[] { 0.1, 0.1, 0.1, 0.1 };
		public static final double[] drivekD = CONFIG.isHammerHead()? new double[] {0.005,0.005,0.005,0.005}:
			new double[] { 0, 0, 0, 0 };
		public static final boolean[] driveInversion = (CONFIG.isHammerHead()
		? new boolean[] { true, false, true, false }
		: new boolean[] { false, true, false, true });
		public static final boolean[] turnInversion = { true, true, true, true };
		// kS
		// public static final double[] kForwardVolts = { 0.26744, 0.31897, 0.27967, 0.2461 };
		public static final double[] kForwardVolts = CONFIG.isHammerHead() ? new double[] { 0.26744, 0.31897, 0.27967, 0.2461 }:
			new double[] {0, 0, 0, 0}; //{0.59395, 0.52681, 0.11097, 0.17914};      //{ 0.2, 0.2, 0.2, 0.2 };
		public static final double[] kBackwardVolts = kForwardVolts;

		//kV
		// public static final double[] kForwardVels = { 2.81, 2.9098, 2.8378, 2.7391 };
		public static final double[] kForwardVels = CONFIG.isHammerHead() ? new double[] { 2.81, 2.9098, 2.8378, 2.7391 }:
			new double[] { 0, 0, 0, 0 };//{2.4114, 2.7465, 2.7546, 2.7412};        //{ 0, 0, 0, 0 };//volts per m/s
		public static final double[] kBackwardVels = kForwardVels;

		//kA
		// public static final double[] kForwardAccels = { 1.1047 / 2, 0.79422 / 2, 0.77114 / 2, 1.1003 / 2 };
		public static final double[] kForwardAccels = { 0, 0, 0, 0 };//{0.31958, 0.33557, 0.70264, 0.46644};    //{ 0, 0, 0, 0 };// volts per m/s^2
		public static final double[] kBackwardAccels = kForwardAccels;

		public static final double autoMaxSpeedMps = 0.35 * 4.4; // Meters / second
		public static final double autoMaxAccelMps2 = mu * g; // Meters / seconds^2
		public static final double autoMaxVolt = 10.0; // For Drivetrain voltage constraint in RobotPath.java
		// The maximum acceleration the robot can achieve is equal to the coefficient of
		// static friction times the gravitational acceleration
		// a = mu * 9.8 m/s^2
		public static final double autoCentripetalAccel = mu * g * 2;

		public static final boolean isGyroReversed = true;

		// PID values are listed in the order kP, kI, and kD
		public static final double[] xPIDController = CONFIG.isHammerHead() ? new double[] { 4, 0.0, 0.0 }
		: new double[] { 2, 0.0, 0.0 };
		public static final double[] yPIDController = xPIDController;
		public static final double[] thetaPIDController = CONFIG.isHammerHead() ? new double[] { 0.10, 0.0, 0.001 }
		: new double[] {0.05, 0.0, 0.00};

		public static final SwerveConfig swerveConfig = new SwerveConfig(wheelDiameterMeters, driveGearing, mu,
		autoCentripetalAccel, kForwardVolts, kForwardVels, kForwardAccels, kBackwardVolts, kBackwardVels,
		kBackwardAccels, drivekP, drivekI, drivekD, turnkP, turnkI, turnkD, turnkS, turnkV, turnkA, turnZeroDeg,
		driveInversion, reversed, driveModifier, turnInversion);

		// public static final Limelight.Transform limelightTransformForPoseEstimation =
		// Transform.BOTPOSE_WPIBLUE;

		// #endregion

		// #region Ports
		//I think all of these are right
		public static final int driveFrontLeftPort = CONFIG.isHammerHead() ? 1 : 1;
		public static final int driveFrontRightPort = CONFIG.isHammerHead() ? 2 : 2;
		public static final int driveBackLeftPort = CONFIG.isHammerHead() ? 3 : 3;
		public static final int driveBackRightPort = CONFIG.isHammerHead() ? 4 : 4;

		public static final int turnFrontLeftPort = CONFIG.isHammerHead() ? 11 : 11;
		public static final int turnFrontRightPort = CONFIG.isHammerHead() ? 12 : 12;
		public static final int turnBackLeftPort = CONFIG.isHammerHead() ? 17 : 13;
		public static final int turnBackRightPort = CONFIG.isHammerHead() ? 14 : 14;
		//to be configured
		public static final int canCoderPortFL = CONFIG.isHammerHead() ? 0 : 1; 
		public static final int canCoderPortFR = CONFIG.isHammerHead() ? 3 : 2; 
		public static final int canCoderPortBL = CONFIG.isHammerHead() ? 2 : 3;
		public static final int canCoderPortBR = CONFIG.isHammerHead() ? 1 : 0; 

		// #endregion

		// #region Command Constants

		public static double kNormalDriveSpeed = 1; // Percent Multiplier	
		public static double kNormalDriveRotation = 0.5; // Percent Multiplier
		public static double kSlowDriveSpeed = 0.4; // Percent Multiplier
		public static double kSlowDriveRotation = 0.250; // Percent Multiplier

		//baby speed values, i just guessed the percent multiplier. TODO: find actual ones we wana use
		public static double kBabyDriveSpeed = 0.3;
		public static double kBabyDriveRotation = 0.2;
		public static double kAlignMultiplier = 1D / 3D;
		public static final double kAlignForward = 0.6;

		public static final double wheelTurnDriveSpeed = 0.0001; // Meters / Second ; A non-zero speed just used to
														// orient the wheels to the correct angle. This
														// should be very small to avoid actually moving the
														// robot.

		public static final double[] positionTolerance = { Units.inchesToMeters(.5), Units.inchesToMeters(.5), 5 }; // Meters,
																										// Meters,
																										// Degrees
		public static final double[] velocityTolerance = { Units.inchesToMeters(1), Units.inchesToMeters(1), 5 }; // Meters,
																													// Meters,
																													// Degrees/Second

		// #endregion
		// #region Subsystem Constants

		// "swerveRadius" is the distance from the center of the robot to one of the
		// modules
		public static final double turnkP_avg = (turnkP[0] + turnkP[1] + turnkP[2] + turnkP[3]) / 4;
		public static final double turnIzone = .1;

		public static final double driveIzone = .1;

		public static final class Autoc {
			public static final RobotConfig robotConfig = new RobotConfig(
					// Mass mass, kg
					ROBOTMASS_KG,
					// double Moment Oof Inertia, kg/mm
					MOI, // ==1
					// ModuleConfig moduleConfig,
					new ModuleConfig(
							// double wheelRadiusMeters,
							swerveRadius,
							// double maxDriveVelocityMPS,
							autoMaxSpeedMps,
							// double wheelCOF,
							mu,
							// DCMotor driveMotor,
							DCMotor.getNEO(4), // FIXME is it 1 for 1 each or 4 for 1 robot
							// double driveGearing,
							driveGearing,
							// double driveCurrentLimit,
							autoMaxVolt,
							// int numMotors
							4),
					// Translation2d... moduleOffsets
					new Translation2d(wheelBase / 2, trackWidth / 2),
					new Translation2d(wheelBase / 2, -trackWidth / 2),
					new Translation2d(-wheelBase / 2, trackWidth / 2),
					new Translation2d(-wheelBase / 2, -trackWidth / 2));
			// public static final ReplanningConfig repConfig = new ReplanningConfig( /*
			// * put in
			// * Constants.Drivetrain.Auto
			// */
			// false, // replan at start of path if robot not at start of path?
			// false, // replan if total error surpasses total error/spike threshold?
			// 1.5, // total error threshold in meters that will cause the path to be
			// replanned
			// 0.8 // error spike threshold, in meters, that will cause the path to be
			// replanned
			// );
			public static final PathConstraints pathConstraints = new PathConstraints(1.54, 6.86, 2 * Math.PI,
					2 * Math.PI); // The constraints for this path. If using a differential drivetrain, the
									// angular constraints have no effect.
		}
	}

	public static final class Limelightc {
		
		public static final class Apriltag {

		}
	}

	public static final class OI {
		public static final class Driver {
			public static final int port = 0;

			public static final int slowDriveButton = Button.kLeftBumper.value;
			public static final int resetFieldOrientationButton = Button.kRightBumper.value;
			public static final int toggleFieldOrientedButton = Button.kStadia.value;

			public static final int y = Button.kY.value;
			public static final int b = Button.kB.value;
			public static final int a = Button.kA.value;
			public static final int x = Button.kX.value;
		}

		public static final class Manipulator {
			public static final int port = 1;
			public static final int Y = Button.kY.value;
        }

		public static final double JOY_THRESH = 0.02;
		public static final double MIN_AXIS_TRIGGER_VALUE = 0.2;// woah, this is high.

	}
	public static final class CoralEffectorc{
        public final static int CORAL_MOTOR_PORT = 30;
        public final static int CORAL_LIMIT_SWITCH_PORT = 0;
        public final static int CORAL_DISTANCE_SENSOR_PORT = 6;
        public final static int CORAL_DISTANCE_SENSOR_DISTANCE = 150; //mm
        public final static double CORAL_INTAKE_ERR = .1;//encoder units - rotations
        public final static double INPUT_FAST_SPEED = 0.07; //TODO: tune this
		public final static double INTAKE_BACKWARDS_SPEED = -0.06; //TODO tune this
        public final static double INPUT_SLOW_SPEED = 0.02; //TODO: tune this
        public final static double OUTPUT_SPEED = 0.4; //TODO: tune this
		public final static double FAST_OUTPUT_SPEED = 0.6;
        public final static double CORAL_EFFECTOR_DISTANCE_SENSOR_OFFSET = -0.3; //TOD: tune this?
        public final static double KP = 2; //TODO: tune this
        public final static double KI = 0;
        public final static double KD = 0;
        public final static double INTAKE_TIME_OUT = 0.5;
        public final static double OUTTAKE_TIME_OUT = 10;
        public final static double MANUAL_INTAKE_TIME_OUT = 1;

        // public final static int LIMIT_SWITCH_PORT = 7; //TODO: Change
    }
	public static final class AlgaeEffectorc {

        //EFFECTOR

        
		public static final int UPPER_MOTOR_PORT = 1; 
		public static final int LOWER_MOTOR_PORT = 33;
        public static final int PINCH_MOTOR_PORT = 3;
        public static final int ARM_MOTOR_PORT = 34;
        public static final int aChannelEnc = 0;
        public static final int bChannelEnc = 1;

		public static final int TOP_ARRAY_ORDER = 0;
		public static final int BOTTOM_ARRAY_ORDER = 1;
		public static final int PINCHER_ARRAY_ORDER = 2;
        public static final int ARM_ARRAY_ORDER = 3;
        //the ArrayOrder variables replaced the ones for the kS since they just indicate the order and are the same for all PID values
        //TODO find these values out 
        
        public static double INTAKE_PINCHER_RPM = 1000;  
        public static double BOTTOM_MOTOR_SPEED = 0.3;

        
        public static double OUTTAKE_PINCHER_RPM = -2100; 
         
        public static double SHOOT_PINCHER_RPM = -2100; 

         
        public static double DEALGAFY_PINCHER_RPM = 1000; 

        public static double RPM_ALLOWED_ERROR = 150;//rpm

        public static final int TBE_CPR = 8192; //Through-Bore Encoder
        public static final double TBE_DPP = 360.0/TBE_CPR; //Degrees per pulse
        public static final boolean invertedTBE = false; //if the encoder needs to read invertedly
        public static final CounterBase.EncodingType encodingType = CounterBase.EncodingType.k2X;
        
        public static final double ARM_CHAIN_GEARING = 16.0/34;// TODO: set to 16.0/34
        public static final double ARM_GEAR_RATIO = 1.0/3;
        //TODO figure the zero out once encoder is on
        public static final double ARM_ZERO_ROT = Units.degreesToRotations(0); //Change for actual robot
        //TODO ask samo for angle to intake algae from pure vertical down
        public static final double ARM_INTAKE_ANGLE = 0;
        //TODO Figure these two out 
        public static final double ARM_RAMP_UP_ANGLE = 0;
        public static final double ARM_OUTTAKE_ANGLE = 0;
        
        //TODO Figure angle for dealgafying
        public static final double ARM_DEALGAFYING_ANGLE = 0;
        //TODO figure out resting angle of the arm while algae inside
        public static final double ARM_HOLDING_ALGAE_ANGLE = 0.0;
        //TODO figure out current threshold for pincher wheels
        public static final double PINCHER_CURRENT_THRESHOLD = 15.0;

        public static final double UPPER_ANGLE_LIMIT = 20;
        public static final double LOWER_ANGLE_LIMIT = -93;
        public static final double ROTATION_TO_DEG = 360;
        public static final double ROTATION_TO_RAD = 2*Math.PI;
        public static final double DEGREES_TO_RADS = Math.PI/180;
        public static final double ARM_DISCONT_DEG = -35;
        public static TrapezoidProfile.Constraints TRAP_CONSTRAINTS;
        public static final double MAX_FF_VEL_RAD_P_S = (Math.PI * .5)/2;
		public static final double MAX_FF_ACCEL_RAD_P_S = (53.728 / 4)/2;
        public static final double ARM_ERROR_MARGIN = 5;

        public static final double ARM_SYS_ID_START_COMMAND_ANGLE = -22; //TODO:

		public static final double armKP = 0.0018;
		public static final double armKI = 0.0;
		public static final double armKD = 0.08;
		public static final double armKS = 0.0;
		public static final double armKV = 0.0;
		public static final double armKA = 0.0;
		public static final double armKG = 0.63;


	}
    
}
