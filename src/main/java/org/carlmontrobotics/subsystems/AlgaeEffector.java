package org.carlmontrobotics.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.carlmontrobotics.lib199.MotorConfig;
//import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;


import static org.carlmontrobotics.RobotContainer.*;

import org.carlmontrobotics.Constants;
import org.carlmontrobotics.RobotContainer;
import org.carlmontrobotics.commands.DealgaficationIntake;
import org.carlmontrobotics.commands.GroundIntakeAlgae;
import org.carlmontrobotics.commands.OuttakeAlgae;
import org.carlmontrobotics.commands.ShootAlgae;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj.Encoder;

import static org.carlmontrobotics.Constants.AlgaeEffectorc.*;
import static org.carlmontrobotics.Constants.*;
import edu.wpi.first.util.sendable.*;


public class AlgaeEffector extends SubsystemBase {

    
    //motors
    private final SparkFlex topMotor = null; //new SparkFlex(UPPER_MOTOR_PORT, MotorType.kBrushless);
    private final SparkFlex bottomMotor = null; //new SparkFlex(LOWER_MOTOR_PORT, MotorType.kBrushless); 
    private final SparkFlex pincherMotor = null; //new SparkFlex(PINCH_MOTOR_PORT, MotorType.kBrushless);
    private final SparkMax armMotor = MotorControllerFactory.createSparkMax(ARM_MOTOR_PORT, MotorConfig.NEO);

    private SparkFlexConfig pincherMotorConfig = new SparkFlexConfig();
    private SparkFlexConfig bottomMotorConfig = new SparkFlexConfig();
    private SparkFlexConfig topMotorConfig = new SparkFlexConfig();
    private SparkMaxConfig  armMotorConfig = new SparkMaxConfig();
    
    
    private final RelativeEncoder topEncoder = (topMotor != null ? topMotor.getEncoder() : null); //for testing purposes: when testing set unused motors to null and code should still run
    private final RelativeEncoder bottomEncoder = (bottomMotor != null ? bottomMotor.getEncoder() : null);
    private final RelativeEncoder pincherEncoder = (pincherMotor != null ? pincherMotor.getEncoder() : null);
    private final RelativeEncoder armEncoder = (armMotor != null ? armMotor.getEncoder() : null);
    private final AbsoluteEncoder armAbsoluteEncoder = (armMotor != null ? armMotor.getAbsoluteEncoder() : null);

    private final SparkClosedLoopController pidControllerTop = (topMotor != null ? topMotor.getClosedLoopController() : null);
    private final SparkClosedLoopController pidControllerBottom = (bottomMotor != null ? bottomMotor.getClosedLoopController() : null);
    private final SparkClosedLoopController pidControllerPincher = (pincherMotor != null ? pincherMotor.getClosedLoopController() : null);
    private final SparkClosedLoopController pidControllerArm = (armMotor != null ? armMotor.getClosedLoopController() : null);
    
    private final SimpleMotorFeedforward topFeedforward = new SimpleMotorFeedforward(kS[TOP_ARRAY_ORDER], kV[TOP_ARRAY_ORDER], kA[TOP_ARRAY_ORDER]);
    private final SimpleMotorFeedforward bottomFeedforward = new SimpleMotorFeedforward(kS[BOTTOM_ARRAY_ORDER], kV[BOTTOM_ARRAY_ORDER], kA[BOTTOM_ARRAY_ORDER]);
    private final SimpleMotorFeedforward pincherFeedforward = new SimpleMotorFeedforward(kS[PINCHER_ARRAY_ORDER], kV[PINCHER_ARRAY_ORDER], kA[PINCHER_ARRAY_ORDER]);
    AbsoluteEncoderConfig config = new AbsoluteEncoderConfig();
    //for sendable we need this stuff; TODO: remove
    private double armkS = kS[ARM_ARRAY_ORDER];
    private double armkV = kV[ARM_ARRAY_ORDER];
    private double armkA = kA[ARM_ARRAY_ORDER];
    private double armkG = kG[ARM_ARRAY_ORDER];
    private double armkP = Constants.kP[ARM_ARRAY_ORDER];
    private double armkI = Constants.kI[ARM_ARRAY_ORDER];
    private double armkD = Constants.kD[ARM_ARRAY_ORDER];
    private ArmFeedforward armFeedforward = new ArmFeedforward(armkS, armkG, armkV, armkA);
    private void updateFeedforward() {
        armFeedforward = new ArmFeedforward(armkS, armkG, armkV, armkA);
    }
    private void updateArmPID() {
        // Update the arm motor PID configuration with the new values
        armMotorConfig.closedLoop.pid(armkP, armkI , armkD);
        armMotor.configure(armMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
       
    }
    //private SimpleMotorFeedforward armFeedforward = new SimpleMotorFeedforward(armkS, armkV, armkA);
    
    //private final SimpleMotorFeedforward armFeedforward = new SimpleMotorFeedforward(kS[ARM_ARRAY_ORDER], kV[ARM_ARRAY_ORDER], kA[ARM_ARRAY_ORDER]); //change
    //feedforward for arm was added
    


    //Arm Trapezoid Profile
    private TrapezoidProfile armTrapProfile;
    private TrapezoidProfile.State armGoalState = new TrapezoidProfile.State(0,0); //please write down if armGoalState.position is in radians or degrees

    private static double kDt;
    private TrapezoidProfile.State setPoint;
    
    private double armFeedVolts;

    // SYS ID
    private final MutableMeasure<Voltage> voltage = mutable(Volts.of(0));
    private final MutableMeasure<Velocity<Angle>> velocity =
            mutable(RadiansPerSecond.of(0));
    private final MutableMeasure<Angle> distance = mutable(Radians.of(0));
    
    //private final ArmFeedforward armFeed = new ArmFeedforward(kS[ARM_ARRAY_ORDER], kG[ARM_ARRAY_ORDER], kV[ARM_ARRAY_ORDER], kA[ARM_ARRAY_ORDER]);

    //--------------------------------------------------------------------------------------------
    public AlgaeEffector() {
        TRAP_CONSTRAINTS = new TrapezoidProfile.Constraints(
                (MAX_FF_VEL_RAD_P_S), (MAX_FF_ACCEL_RAD_P_S));
        armTrapProfile = new TrapezoidProfile(TRAP_CONSTRAINTS);
        configureMotors();
        kDt = 0.02;
        setPoint = getArmState();
        
        

        SmartDashboard.putData("Arm to Zero Degrees",new InstantCommand(() -> setArmTarget(0)));
        SmartDashboard.putData("Arm to Shooting Angle",new InstantCommand(() -> setArmTarget(Constants.AlgaeEffectorc.ARM_SHOOT_ANGLE)));
        SmartDashboard.putData("Arm to Intake Angle",new InstantCommand(() -> setArmTarget(Constants.AlgaeEffectorc.ARM_INTAKE_ANGLE)));
        SmartDashboard.putData("Arm to Dealgafication Angle",new InstantCommand(() -> setArmTarget(Constants.AlgaeEffectorc.ARM_DEALGAFYING_ANGLE)));
        SmartDashboard.putData("Arm to Resting While Intake Angle",new InstantCommand(() -> setArmTarget(Constants.AlgaeEffectorc.ARM_RESTING_ANGLE_WHILE_INTAKE_ALGAE)));
        SmartDashboard.putData("Arm to Ramp Up Angle Angle",new InstantCommand(() -> setArmTarget(Constants.AlgaeEffectorc.ARM_RAMP_UP_ANGLE)));
        SmartDashboard.putData("Dealgafication", new DealgaficationIntake(this));
        SmartDashboard.putData("Intake Algae", new GroundIntakeAlgae(this));
        SmartDashboard.putData("Outtake Algae", new OuttakeAlgae(this));
        SmartDashboard.putData("Shoot Algae", new ShootAlgae(this));

    }
    //----------------------------------------------------------------------------------------

    private void configureMotors () {
        topMotorConfig.closedLoop.pid(
            Constants.kP[TOP_ARRAY_ORDER],
            Constants.kI[TOP_ARRAY_ORDER],
            Constants.kD[TOP_ARRAY_ORDER]
            ).feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        if (topMotor != null) {
            topMotor.configure(topMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }
       
        
        bottomMotorConfig.closedLoop.pid(
            Constants.kP[BOTTOM_ARRAY_ORDER],
            Constants.kI[BOTTOM_ARRAY_ORDER],
            Constants.kD[BOTTOM_ARRAY_ORDER]
            ).feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        if (bottomMotor != null) {
            bottomMotor.configure(bottomMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }
           
    
        pincherMotorConfig.closedLoop.pid(
            Constants.kP[PINCHER_ARRAY_ORDER],
            Constants.kI[PINCHER_ARRAY_ORDER],
            Constants.kD[PINCHER_ARRAY_ORDER]
            ).feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        if (pincherMotor != null) {
            pincherMotor.configure(pincherMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }
        

        //todo
        armMotorConfig.closedLoop.pid(
            armkP, //change to: Constants.kP[ARM_ARRAY_ORDER]
            armkI, //change to:  Constants.kI[ARM_ARRAY_ORDER]
            armkD  // Constants.kd[ARM_ARRAY_ORDER]
            ).feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        armMotorConfig.absoluteEncoder.zeroOffset(ARM_ZERO_ROT);
        armMotor.configure(pincherMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        armMotorConfig.idleMode(IdleMode.kBrake);
        armMotorConfig.closedLoop.pid(
            Constants.kP[ARM_ARRAY_ORDER],
            Constants.kI[ARM_ARRAY_ORDER],
            Constants.kD[ARM_ARRAY_ORDER]
            ).feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        //armMotorConfig.encoder.positionConversionFactor(ROTATION_TO_DEG);
        if (armMotor != null) {
            armMotor.configure(armMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }
        
        
        
        
    }

    public void setTopRPM(double toprpm) {
        if (topMotor != null) {
            pidControllerTop.setReference(toprpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
            topFeedforward.calculate(toprpm);
        }
        
        //ALL THIS DOES IS RETURN THE CALCULATED VOLTAGE TO ADD!! IT DOES NOT DO ANYTHING!!
        //also, only the arm generally needs feedforward - unless you have a flywheel.
    }

    public void setBottomRPM(double bottomrpm) {
        if (bottomMotor != null) {
            pidControllerBottom.setReference(bottomrpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
            bottomFeedforward.calculate(bottomrpm);
        }
        
    }

    public void setPincherRPM(double pincherrpm) {
        if (pincherMotor != null) {
            pidControllerPincher.setReference(pincherrpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
            pincherFeedforward.calculate(pincherrpm); 
        }
           
    }
    //arm methods

    //drives arm from set point to goal position
    public void setArmPosition() {
        
        setPoint = getArmState();
        armGoalState = armTrapProfile.calculate(kDt, setPoint, armGoalState); 

        armFeedVolts = armFeedforward.calculate(Units.degreesToRadians(armGoalState.position), armGoalState.velocity);
        if ((getArmPos() < LOWER_ANGLE_LIMIT)
             || (getArmPos() > UPPER_ANGLE_LIMIT)) {
            armFeedVolts = armFeedforward.calculate(Units.degreesToRadians(getArmPos()), 0);

        }
        //System.out.println(armFeedVolts);
        if (armMotor != null) {
            pidControllerArm.setReference(armGoalState.position, ControlType.kPosition,ClosedLoopSlot.kSlot0, armFeedVolts);
        }
        
        //((setPoint.position),ControlType.kPosition,armFeedVolts);
    }
    
    // // use trapezoid 
    public void setArmTarget(double targetPos){
        armGoalState.position = getArmClampedGoal(targetPos); 
        armGoalState.velocity = 0;
    }



    // //returns the arm position and velocity based on encoder and position 
    public TrapezoidProfile.State getArmState(){
        TrapezoidProfile.State armState = new TrapezoidProfile.State(getArmPos(), getArmVel());
        return armState;
    }
    // //overload

    
    public boolean armAtGoal(){
        return Math.abs(getArmPos()-armGoalState.position) <= ARM_ERROR_MARGIN;
    }

    
    public double getArmClampedGoal(double goalAngle) {
        return MathUtil.clamp(
            MathUtil.inputModulus(goalAngle, ARM_DISCONT_DEG, 
                ARM_DISCONT_DEG + 360),
                LOWER_ANGLE_LIMIT, UPPER_ANGLE_LIMIT
        );
    }
    

    public double getArmPos() {
        //figures out the position of the arm in degrees based off pure vertical down
        //TODO update the arm to get in degrees after someone will figure out what the .getPosition gets for the TBE

        return Units.rotationsToDegrees(armAbsoluteEncoder.getPosition() * ARM_CHAIN_GEARING); 

    }
   
    public void stopArm() {
        if (armMotor != null) {
            armMotor.set(0);
        }
       
    }


    public double getArmVel() {
        return armAbsoluteEncoder.getVelocity();
    }

    public void runRPM() {
        //TODO: Change RPM according to design
        setTopRPM(1000);
        setBottomRPM(2100);
        setPincherRPM(2100);
    }

    public void stopMotors() {
        setTopRPM(0);
        setBottomRPM(0);
        setPincherRPM(0);
    }

    public boolean checkIfAtTopRPM(double rpm) {
        return Math.abs(topEncoder.getVelocity()-rpm) <= RPM_ALLOWED_ERROR;
    }

    public boolean checkIfAtBottomRPM(double rpm) {
        return Math.abs(bottomEncoder.getVelocity()-rpm) <= RPM_ALLOWED_ERROR;
    }

    public void setMotorSpeed(double topSpeed, double bottomSpeed, double pincherSpeed) {
        if (topMotor != null) {
            topMotor.set(topSpeed);
        }
        if (bottomMotor != null) {
            bottomMotor.set(bottomSpeed);
        }
        if (pincherMotor != null) {
            pincherMotor.set(pincherSpeed);
        }
        
    }

    public boolean isAlgaeIntaked() {
        return pincherMotor.getOutputCurrent() > PINCHER_CURRENT_THRESHOLD;
    }


    


    @Override
    public void periodic() {
        //armMotor.set(0.1);
       
        setArmPosition();
        
        SmartDashboard.putNumber("Arm Angle", getArmPos());
        SmartDashboard.putNumber("Raw Arm Angle",Units.rotationsToDegrees(armAbsoluteEncoder.getPosition() * ARM_CHAIN_GEARING)-20);
        armMotor.configure(armMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        if (pincherMotor != null) {
            SmartDashboard.putBoolean("Algae Intaked?", isAlgaeIntaked());
        }
        
        SmartDashboard.putNumber("Arm Velocity", getArmVel());
        SmartDashboard.putData("Qualistatic Forward", sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        SmartDashboard.putData("Dynamic Forwards", sysIdDynamic(SysIdRoutine.Direction.kForward));
        SmartDashboard.putData("Qualistatic Backward", sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        SmartDashboard.putData("Dynamic Backward", sysIdDynamic(SysIdRoutine.Direction.kReverse));

        

        //ARM PID values
        //kS, kV, kA and kG , kP, kI, kD

    }
    private SysIdRoutine.Config defaultSysIdConfig = new SysIdRoutine.Config(
            Volts.of(1).per(Seconds.of(1)), Volts.of(2), Seconds.of(10));

    public void logMotor(SysIdRoutineLog log) {
        log.motor("armMotorMaster")
                .voltage(voltage.mut_replace(armMotorMaster.getBusVoltage()
                        * armMotorMaster.getAppliedOutput(), Volts))
                .angularVelocity(velocity.mut_replace(
                        armMasterEncoder.getVelocity(), RadiansPerSecond))
                .angularPosition(distance
                        .mut_replace(armMasterEncoder.getPosition(), Radians));
    }

    private final SysIdRoutine routine = new SysIdRoutine(defaultSysIdConfig,
            new SysIdRoutine.Mechanism(this::driveMotor, this::logMotor, this));

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> armMasterEncoder.setZeroOffset(0)),
                routine.quasistatic(direction));
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> armMasterEncoder.setZeroOffset(0)),
                routine.dynamic(direction));
    }

    public void initSendable(SendableBuilder builder){
       super.initSendable(builder); 
       builder.addDoubleProperty("arm kS", () -> armkS, (value) -> { armkS = value; updateFeedforward(); });
       builder.addDoubleProperty("arm kV", ()-> armkV, (value) -> { armkV = value; updateFeedforward(); });
       builder.addDoubleProperty("arm kA", ()-> armkA, (value) -> { armkA = value; updateFeedforward(); } );
       builder.addDoubleProperty("arm kG", ()-> armkG, (value) -> { armkG = value; updateFeedforward(); } );
       builder.addDoubleProperty("arm kP", () -> armkP , (value) -> { armkP = value; updateArmPID(); });
       builder.addDoubleProperty("arm kI", () -> armkI , (value) -> { armkI = value; updateArmPID(); });
       builder.addDoubleProperty("arm kD", () -> armkD, (value) -> { armkD = value; updateArmPID(); });
    
    }

}
