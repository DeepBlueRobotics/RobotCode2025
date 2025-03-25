package org.carlmontrobotics.subsystems;


import  edu.wpi.first.units.measure.MutAngle;
import  edu.wpi.first.units.measure.MutAngularVelocity;
import  edu.wpi.first.units.measure.MutDistance;
import  edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVelocity;
import  edu.wpi.first.units.measure.MutVoltage;
import  edu.wpi.first.units.Measure;
import  edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.VoltageUnit;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.carlmontrobotics.lib199.MotorConfig;
//import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;


import static org.carlmontrobotics.RobotContainer.*;

import java.util.function.BooleanSupplier;

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
import edu.wpi.first.math.system.plant.DCMotor;
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
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj.Encoder;

import static org.carlmontrobotics.Constants.AlgaeEffectorc.*;
import static org.carlmontrobotics.Constants.*;
import edu.wpi.first.util.sendable.*;


public class AlgaeEffector extends SubsystemBase {

    private double armGoal = LOWER_ANGLE_LIMIT;
    private double armMaxVelocityDegreesPerSecond;


    private double upperLimitAdjustmentVoltage = -0.2;
    private double lowerLimitAdjustmentVoltage = 0.2;
    //motors
     
    private final SparkFlex pincherMotor = null; //new SparkFlex(PINCH_MOTOR_PORT, MotorType.kBrushless);
    private final SparkMax armMotor = MotorControllerFactory.createSparkMax(ARM_MOTOR_PORT, MotorConfig.NEO);

    private SparkFlexConfig pincherMotorConfig = new SparkFlexConfig();
    
    private SparkMaxConfig  armMotorConfig = new SparkMaxConfig();
    
    
    
    private final RelativeEncoder pincherEncoder = (pincherMotor != null ? pincherMotor.getEncoder() : null);
    private final RelativeEncoder armEncoder = (armMotor != null ? armMotor.getEncoder() : null);
    private final AbsoluteEncoder armAbsoluteEncoder = (armMotor != null ? armMotor.getAbsoluteEncoder() : null);

    
    private final SparkClosedLoopController pidControllerPincher = (pincherMotor != null ? pincherMotor.getClosedLoopController() : null);
    private final SparkClosedLoopController pidControllerArm = (armMotor != null ? armMotor.getClosedLoopController() : null);
    
    
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
        System.out.println("kG" + armkG+"*********************");
    }
    private void updateArmPID() {
        // Update the arm motor PID configuration with the new values
        armMotorConfig.closedLoop.pid(armkP, armkI , armkD).maxMotion.maxVelocity(armMaxVelocityDegreesPerSecond).maxAcceleration(1000);
        armMotor.configure(armMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
       
    }

    //private SimpleMotorFeedforward armFeedforward = new SimpleMotorFeedforward(armkS, armkV, armkA);
    
    //private final SimpleMotorFeedforward armFeedforward = new SimpleMotorFeedforward(kS[ARM_ARRAY_ORDER], kV[ARM_ARRAY_ORDER], kA[ARM_ARRAY_ORDER]); //change
    //feedforward for arm was added
    


    //Arm Trapezoid Profile
    private TrapezoidProfile armTrapProfile;
    private TrapezoidProfile.State armGoalState = new TrapezoidProfile.State(0,0); //please write down if armGoalState.position is in radians or degrees

    private static double kDt;
    private TrapezoidProfile.State currentPosition;
    
    private double armFeedVolts;

    // SYS ID
    // private final MutableMeasure<Voltage> voltage = mutable(Volts.of(0));
    // private final MutableMeasure<Velocity<Angle>> velocity =
    //         mutable(RadiansPerSecond.of(0));
    // private final MutableMeasure<Angle> distance = mutable(Radians.of(0));
    
    //private final ArmFeedforward armFeed = new ArmFeedforward(kS[ARM_ARRAY_ORDER], kG[ARM_ARRAY_ORDER], kV[ARM_ARRAY_ORDER], kA[ARM_ARRAY_ORDER]);
    private final MutVoltage voltage = Volts.mutable(0);
    private final MutAngle distance = Radians.mutable(0);
    private final MutAngularVelocity velocity = RadiansPerSecond.mutable(0);

    //--------------------------------------------------------------------------------------------
    public AlgaeEffector() {
        TRAP_CONSTRAINTS = new TrapezoidProfile.Constraints(
                (MAX_FF_VEL_RAD_P_S), (MAX_FF_ACCEL_RAD_P_S));
        armTrapProfile = new TrapezoidProfile(TRAP_CONSTRAINTS);
        configureMotors();
        kDt = 0.02;
        currentPosition = getArmState();
        setArmTarget(0);
        updateArmPID(); 

        SmartDashboard.putData("Arm to Zero Degrees",new InstantCommand(() -> setArmTarget(0)));
        
        SmartDashboard.putData("Arm to Intake Angle",new InstantCommand(() -> setArmTarget(Constants.AlgaeEffectorc.ARM_INTAKE_ANGLE)));
        SmartDashboard.putData("Arm to Dealgafication Angle",new InstantCommand(() -> setArmTarget(Constants.AlgaeEffectorc.ARM_DEALGAFYING_ANGLE)));
        ;
        SmartDashboard.putData("Arm to Ramp Up Angle Angle",new InstantCommand(() -> setArmTarget(Constants.AlgaeEffectorc.ARM_RAMP_UP_ANGLE)));
        SmartDashboard.putData("Dealgafication", new DealgaficationIntake(this));
        SmartDashboard.putData("Intake Algae", new GroundIntakeAlgae(this));
        SmartDashboard.putData("Outtake Algae", new OuttakeAlgae(this));
        SmartDashboard.putData("Shoot Algae", new ShootAlgae(this));
        SmartDashboard.putData("UPDATE COMMAND",new InstantCommand(()->{updateArmPID();updateFeedforward();}));

        SmartDashboard.putData("Quasistatic Forward", sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        SmartDashboard.putData("Dynamic Forwards", sysIdDynamic(SysIdRoutine.Direction.kForward));
        SmartDashboard.putData("Quasistatic Backward", sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        SmartDashboard.putData("Dynamic Backward", sysIdDynamic(SysIdRoutine.Direction.kReverse));

    }
    //----------------------------------------------------------------------------------------

    private void configureMotors () {
        
    
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
            ).feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        armMotorConfig.absoluteEncoder.zeroOffset(ARM_ZERO_ROT);
        armMotorConfig.absoluteEncoder.zeroCentered(true);
        armMotorConfig.absoluteEncoder.inverted(true);
        //armMotorConfig.inverted(true);
        // armMotor.configure(pincherMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        armMotorConfig.idleMode(IdleMode.kBrake);
        // armMotorConfig.closedLoop.pid(
        //     Constants.kP[ARM_ARRAY_ORDER],
        //     Constants.kI[ARM_ARRAY_ORDER],
        //     Constants.kD[ARM_ARRAY_ORDER]
        //     ).feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        armMotorConfig.encoder.positionConversionFactor(ROTATION_TO_DEG);
        armMotorConfig.absoluteEncoder.positionConversionFactor(ROTATION_TO_DEG);
        armMotorConfig.absoluteEncoder.velocityConversionFactor(6); // 6 is rotations/min to degrees/second
        if (armMotor != null) {
            armMotor.configure(armMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
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
    /* 
    public void setArmPosition() {
        
        currentPosition = getArmState();
        // System.out.println("origgoalpos: "+armGoalState.position);
        TrapezoidProfile.State calculateTo = armTrapProfile.calculate(kDt, currentPosition, armGoalState); 

        updateFeedforward();
        // armFeedVolts = armFeedforward.calculate(calculateTo.position, 0);
        // armFeedVolts = armFeedforward.calculate(currentPosition.position, 0);
        double s = (getArmPos()>calculateTo.position) ? -armkS : armkS;
        //armFeedVolts = s + Math.cos(Units.degreesToRadians(getArmPos()))*armkG;

        System.out.println("### feed volts: "+armFeedVolts);

        if ((getArmPos() < LOWER_ANGLE_LIMIT)
             || (getArmPos() > UPPER_ANGLE_LIMIT)) {
            
            // armFeedVolts = armFeedforward.calculate(Units.degreesToRadians(getArmPos()), 0);
            // double s = (getArmPos()>calculateTo.position) ? -armkS : armkS;
            // armFeedVolts = s + Math.cos(Units.degreesToRadians(getArmPos()))*armkG;
            // armFeedVolts=1;
            System.out.println("### feed volts: "+armFeedVolts);
        }

        // System.out.println("outvolts: "+armFeedVolts);
        // System.out.println("err: "+(calculateTo.position-currentPosition.position));
        if (true || armMotor != null) {
            pidControllerArm.setReference(armGoalState.position, ControlType.kPosition,ClosedLoopSlot.kSlot0, armFeedVolts);
        }

        // pidControllerArm.setReference(0, ControlType.kPosition,ClosedLoopSlot.kSlot0, armFeedVolts);
        
        //((setPoint.position),ControlType.kPosition,armFeedVolts);
    }
    */

    public double manualFF(double goalRad){
        double cvel = Units.degreesToRadians(Units.rotationsToDegrees(armAbsoluteEncoder.getVelocity()));
        double cpos = Units.degreesToRadians(getArmPos());
        double err = goalRad - cpos;

        double ks_volts = armkS * (Math.abs(err)/err);
        double kg_volts = Math.cos(goalRad) * armkG;

        double kv_calc = 1 / (DCMotor.getNEO(1).withReduction(ARM_GEAR_RATIO).KvRadPerSecPerVolt);// 1 / (rad/s/v) = Vs/rad
        // double kv_volts = kv_calc * cvel;
        double kv_volts=0;

        armFeedVolts = ks_volts + kg_volts + kv_volts;

        return armFeedVolts;
    }
    // // use trapezoid 
    public void setArmTarget(double targetPos){
        // 

        armGoal = targetPos;
        getArmClampedGoal(targetPos); 
        //armGoalState.velocity = 0;


        if (armMotor != null) {
            
            //armFeedforward.calculate(Units.degreesToRadians(armGoal), -0.00001*(armGoal- getArmPos()));
            armFeedVolts = manualFF(Units.degreesToRadians(armGoal));
            // System.out.println("feedVolts: "+ armFeedVolts);
            pidControllerArm.setReference(Units.degreesToRotations(armGoal), ControlType.kPosition, ClosedLoopSlot.kSlot0, armFeedVolts);

        }
        
        
    }



    // //returns the arm position and velocity based on encoder and position 
    public TrapezoidProfile.State getArmState(){
        TrapezoidProfile.State armState = new TrapezoidProfile.State(getArmPos(), getArmVel());
        return armState;
    }
    // //overload

    
    public boolean armAtGoal(){
        return Math.abs(getArmPos()-armGoal) <= ARM_ERROR_MARGIN;
    }

    
    public double getArmClampedGoal(double goalAngle) {
        return MathUtil.clamp(
            MathUtil.inputModulus(goalAngle, -180, 
                180),
                LOWER_ANGLE_LIMIT, UPPER_ANGLE_LIMIT
        );
    }
    

    public double getArmPos() {
        //figures out the position of the arm in degrees based off pure vertical down
        //TODO update the arm to get in degrees after someone will figure out what the .getPosition gets for the TBE

        return armAbsoluteEncoder.getPosition(); 

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
        
        setPincherRPM(2100);
    }

    public void stopPincherMotor() {
        setPincherRPM(0);
    }

    

    public void setMotorSpeed(double topSpeed, double bottomSpeed, double pincherSpeed) {
        
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

        // armkI = SmartDashboard.getNumber("arm kI", 0);
        // armkP = SmartDashboard.getNumber("arm kP", 0);
        // armkD = SmartDashboard.getNumber("arm kD", 0);
        //updateArmPID();
        
       
        // setArmPosition();
        // System.out.println(".");
        // System.out.println("feedVolts: "+ armFeedVolts);
        
        SmartDashboard.putNumber("Arm Angle", getArmPos());
        SmartDashboard.putNumber("goal position", armGoalState.position);
        //SmartDashboard.putNumber("Raw Arm Angle",armAbsoluteEncoder.getPosition());
        System.out.println("_feedVolts: "+ armFeedVolts);
        System.out.println("pid: "+armkP+", "+armkI+", "+armkD+" | ff sg: "+armkS+", "+armkG);
        System.out.println("goal angle:" + armGoal);
        
        setArmTarget(armGoal);
            
        
        if (getArmPos() < LOWER_ANGLE_LIMIT) { //if the arm is below this angle limit it is supposed to stop applying voltage
        
            //armMotor.set(0);
            System.out.println("arm past lower limit!");
            armMotor.set(0);
            if (Math.abs(getArmPos()-LOWER_ANGLE_LIMIT) > ARM_ERROR_MARGIN){ //this is supposed to make it so that if the arm gets too far from the angle limit it applies a reverse voltage to slow it down
                armMotor.set(0.02 * armAbsoluteEncoder.getVelocity() + lowerLimitAdjustmentVoltage);
            }
            
        }
        if (getArmPos() > UPPER_ANGLE_LIMIT) {
            
            armMotor.set(0);
            System.out.println("arm past upper limit");
            if (Math.abs(getArmPos() - LOWER_ANGLE_LIMIT) > ARM_ERROR_MARGIN) {
                armMotor.set(-0.02 * armAbsoluteEncoder.getVelocity() + upperLimitAdjustmentVoltage);
            }
            
            
        }
        
        
        //System.out.println("!!!!!!!!" + getArmPos());
        // System.out.println("outvolts: "+ armFeedVolts);
        /*
        SmartDashboard.putNumber("Raw Arm Angle",Units.rotationsToDegrees(armAbsoluteEncoder.getPosition()));
        //armMotor.configure(armMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        if (pincherMotor != null) {
            SmartDashboard.putBoolean("Algae Intaked?", isAlgaeIntaked());
        }
        
        SmartDashboard.putNumber("Arm Velocity", getArmVel());
        

        

        //ARM PID values
        //kS, kV, kA and kG , kP, kI, kD
        */

    }
    

    private SysIdRoutine.Config defaultSysIdConfig = new SysIdRoutine.Config(Volts.of(10).per(Seconds),
    Volts.of(1.5), Seconds.of(15));

    // private SysIdRoutine.Config defaultSysIdConfig = new SysIdRoutine.Config(
    //         Velocity<VoltageUnit>.ofBaseUnits(1, Volts), Volts.of(2), Seconds.of(10));

    public void logMotor(SysIdRoutineLog log) {
        
        log.motor("armMotor")
                .voltage(voltage.mut_replace(armMotor.getBusVoltage()
                        * armMotor.getAppliedOutput(), Volts))
                .angularVelocity(velocity.mut_replace(
                       armAbsoluteEncoder.getVelocity(), DegreesPerSecond))
                .angularPosition(distance
                        .mut_replace(armAbsoluteEncoder.getPosition(), Degrees));
    }
    
    public void driveMotor(Voltage volts) {
        
        armMotor.setVoltage(volts.in(Volts));
        System.out.println("drive");
        // if (armAbsoluteEncoder.getPosition() > UPPER_ANGLE_LIMIT 
        // || armAbsoluteEncoder.getPosition() < LOWER_ANGLE_LIMIT) {
        //     armMotor.setVoltage(0);
        // }
    
    }

    

    private final SysIdRoutine routine = new SysIdRoutine(defaultSysIdConfig,
            new SysIdRoutine.Mechanism(this::driveMotor, this::logMotor, this));
    public Command canStartSysId(){
        return new WaitUntilCommand(
            (BooleanSupplier) () -> this.getArmPos() < ARM_SYS_ID_START_COMMAND_ANGLE);
        
    }
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
       
        
        return 
            new SequentialCommandGroup(
                canStartSysId(), 
                routine.quasistatic(direction).onlyWhile(()->{
                    return (getArmPos() > LOWER_ANGLE_LIMIT && getArmPos() < UPPER_ANGLE_LIMIT);
                    
                })
            );
            
            
        
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return 
            new SequentialCommandGroup(
                canStartSysId(), 
                routine.dynamic(direction).onlyWhile(()->{
                    return (getArmPos() > LOWER_ANGLE_LIMIT && getArmPos() < UPPER_ANGLE_LIMIT);
                    
                })
            );
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
       builder.addDoubleProperty("Upper Voltage Counter limit", () -> upperLimitAdjustmentVoltage, (value) -> { upperLimitAdjustmentVoltage = value;});
       builder.addDoubleProperty("Lower Voltage Counter limit", () -> lowerLimitAdjustmentVoltage, (value) -> { lowerLimitAdjustmentVoltage = value;});
       builder.addDoubleProperty("Set Arm Max Velocity", () -> armMaxVelocityDegreesPerSecond, (value) -> { armMaxVelocityDegreesPerSecond = value; updateArmPID(); });
       builder.addDoubleProperty("arm angle (degrees)", () -> getArmPos(), null);
       builder.addDoubleProperty("output volts", () -> armMotor.getAppliedOutput()*armMotor.getBusVoltage(), null);
       builder.addDoubleProperty("Set Goal Angle in Degrees", () -> armGoal, (value) -> {setArmTarget(value); });
    }


}
