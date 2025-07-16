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
import org.carlmontrobotics.commands.AlgaeCommands.ManualDynamicForArm;
import org.carlmontrobotics.commands.AlgaeCommands.ManualQuasistaticForArm;


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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj.Encoder;

import static org.carlmontrobotics.Constants.AlgaeEffectorc.*;
import static org.carlmontrobotics.Constants.OI.Driver.a;
import static org.carlmontrobotics.Constants.*;
import edu.wpi.first.util.sendable.*;


public class AlgaeEffector extends SubsystemBase {

    private double armGoal = LOWER_ANGLE_LIMIT; 
    private double clampedArmGoal = LOWER_ANGLE_LIMIT;
    private double armMaxVelocityDegreesPerSecond = 720; //change if nessesary

    //This is for the manual sysID methods
    private final Timer timer = new Timer();
    private final Timer timer2 = new Timer();
    private double TEST_DURATION = 2.0;
    private double voltsApplied = 0;

    private double upperLimitAdjustmentVoltage = -0.2;
    private double lowerLimitAdjustmentVoltage = 0.2;
    private double armFeedVolts;
    private double armPIDVolts;

    private final SparkMax armMotor = MotorControllerFactory.createSparkMax(ARM_MOTOR_PORT, MotorConfig.NEO);

    private SparkMaxConfig  armMotorConfig = new SparkMaxConfig();

    private final RelativeEncoder armEncoder = (armMotor != null ? armMotor.getEncoder() : null);
    private final AbsoluteEncoder armAbsoluteEncoder = (armMotor != null ? armMotor.getAbsoluteEncoder() : null);

    
    private final SparkClosedLoopController sparkPIDControllerArm = (armMotor != null ? armMotor.getClosedLoopController() : null);
    private PIDController pidControllerArm = new PIDController(0, 0, 0);
    AbsoluteEncoderConfig config = new AbsoluteEncoderConfig();

    //for sendable we need this stuff
    private double KS = armKS;
    private double KV = armKV;
    private double KA = armKA;
    private double KG = armKG;
    private double KP = armKP;
    private double KI = armKI;
    private double KD = armKD;
    private ArmFeedforward armFeedforward = new ArmFeedforward(KS, KG, KV, KA);
    
    
    /**
     * Updates ArmFeedForward
     * For testing purely
     */
    private void updateFeedforward() {
        armFeedforward = new ArmFeedforward(KS, KG, KV, KA);
        //System.out.println("kG" + armkG+"*********************");
    }

    /**
     * Updates wpilib armPID
     * For testing purely
     */
    private void updateArmPID() {
        // Update the arm motor PID configuration with the new values
        pidControllerArm = new PIDController(KP, KI, KD);
        // armMotorConfig.closedLoop.pid(KP, KI , KD).maxMotion.maxVelocity(armMaxVelocityDegreesPerSecond).maxAcceleration(100);
        // armMotor.configure(armMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
       
    }
    //TrapezoidProfile components
    private double armGoalVelocity = 0;
    private final double dT = 0.02; // 20ms
    private TrapezoidProfile.Constraints armTrapConstraints = new TrapezoidProfile.Constraints(armMaxVelocityDegreesPerSecond, 100); 
    private TrapezoidProfile.State armSetPoint = new TrapezoidProfile.State(getArmPos(), getArmVel()); //This represents the arm's current state but is also used for the arm's calculated state in dT seconds
    private TrapezoidProfile.State armGoalState = new TrapezoidProfile.State(armGoal, armGoalVelocity);
    private TrapezoidProfile armTrapProfile = new TrapezoidProfile(armTrapConstraints);
    
    
    

    //--------------------------------------------------------------------------------------------
    public AlgaeEffector() {
        
        configureMotors();
        
        setArmPosition(0);
        updateArmPID(); 
        // SmartDashboard.putData("Algae Effector", this);
        // SmartDashboard.putData("Arm to Zero Degrees",new InstantCommand(() -> setArmPosition(0)));
       
        // SmartDashboard.putData("Arm to Intake Angle",new InstantCommand(() -> setArmPosition(Constants.AlgaeEffectorc.ARM_INTAKE_ANGLE)));
        // SmartDashboard.putData("Arm to Dealgafication Angle",new InstantCommand(() -> setArmPosition(Constants.AlgaeEffectorc.ARM_DEALGAFYING_ANGLE)));
        
       
        // SmartDashboard.putData("UPDATE COMMAND",new InstantCommand(()->{updateArmPID();updateFeedforward();}));

        // SmartDashboard.putData("(MANUAL) Dynamic FF test", new ManualDynamicForArm(this)); 
        // SmartDashboard.putData("(MANUAL) Quasistatic FF test", new ManualQuasistaticForArm(this)); 


    }
    //----------------------------------------------------------------------------------------
    /**
     * This sets the settings for the motors and encoders by setting the PID values and other settings
     */
    private void configureMotors () { 
        //TODO: set these to the constants once the values are found
        armMotorConfig.closedLoop.pid(
            KP, 
            KI,  
            KD 
            ).feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        
        
        // armMotorConfig.idleMode(IdleMode.kBrake);
        armMotorConfig.inverted(true);
        armMotorConfig.absoluteEncoder.zeroOffset(ARM_ZERO_ROT);
        armMotorConfig.absoluteEncoder.zeroCentered(true);
        armMotorConfig.absoluteEncoder.inverted(false);
        // armMotorConfig.softLimit.forwardSoftLimit(UPPER_ANGLE_LIMIT);
        // armMotorConfig.softLimit.forwardSoftLimitEnabled(true);
        // armMotorConfig.softLimit.reverseSoftLimit(LOWER_ANGLE_LIMIT);
        // armMotorConfig.softLimit.reverseSoftLimitEnabled(true);
        armMotorConfig.encoder.positionConversionFactor(ROTATION_TO_DEG * ARM_CHAIN_GEARING);
        armMotorConfig.absoluteEncoder.positionConversionFactor(ROTATION_TO_DEG * ARM_CHAIN_GEARING);
        armMotorConfig.absoluteEncoder.velocityConversionFactor(6 * ARM_CHAIN_GEARING); // 6 is rotations/min to degrees/second

        if (armMotor != null) {
            armMotor.configure(armMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        }
        
    }
    /**
     * Uses wpilib controller instead of rev to avoid losing conversion factor
     * @param targetPos goal in degrees
     */
    public void setArmPosition_2(double targetPos) {
        clampedArmGoal = getArmClampedGoal(targetPos);
        armFeedVolts = armFeedforward.calculate(Units.degreesToRadians(clampedArmGoal), clampedArmGoal - getArmPos());
        armPIDVolts = pidControllerArm.calculate(getArmPos(), clampedArmGoal);
        armMotor.setVoltage(armFeedVolts + armPIDVolts);
    }

    public void setArmPosition(double targetPos){ //this method takes in an angle and sets the arm to that angle 
        
        armFeedVolts = armFeedforward.calculate(Units.degreesToRadians(clampedArmGoal), 0.00001*(armGoal- getArmPos()));//this calculates the amount of voltage needed to move the arm
        // sparkPIDControllerArm.setReference(clampedArmGoal, ControlType.kPosition, ClosedLoopSlot.kSlot0, armFeedVolts); //This moves the arm to the goal angle and uses PID 

    }
    /**
     * Moves arm to position using rev controller integrated into the motor
     * @param targetPos goal in degrees
     */
    public void setArmTarget(double targetPos){
        armGoal = targetPos;
        clampedArmGoal = getArmClampedGoal(armGoal); //This takes in the inputted armgoal that was set to targetPos and clamps it so that it can't be outside the safe range
        armGoalState = new TrapezoidProfile.State(clampedArmGoal, armGoalVelocity); //this sets the goal state for trapezoidprofile
    }
    /** move arm to position using TrapezoidProfile 
     * @param targetPos goal in Degrees
    */
    public void setArmTrapPosition(double targetPos){ //run this method in periodic()
        setArmTarget(targetPos); //sets the arm goal to the target position

        armSetPoint = armTrapProfile.calculate(dT, armSetPoint, armGoalState); //sets the current state of the arm to its current position and velocity
        armFeedVolts = armFeedforward.calculate(Units.degreesToRadians(armSetPoint.position), Units.degreesToRadians(armSetPoint.velocity));//this calculates the amount of voltage needed to move the arm
        // sparkPIDControllerArm.setReference(armSetPoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, armFeedVolts); //This moves the arm to the goal angle and uses PID 
        
    }
    /**
     * Checks
     * @return if the arm is at its goal position with the error margin as the tolerance range
     */
    public boolean armAtGoal(){
        return Math.abs(getArmPos()-armGoal) <= ARM_ERROR_MARGIN;
    }

    /**
     * method takes in the input angle and sets it to the range given(Lower angle limit to upper angle limit) 
     * @param goalAngle goal in degrees
     * @return clampedGoal in degrees
     */
    public double getArmClampedGoal(double goalAngle) {
        return MathUtil.clamp(
            MathUtil.inputModulus(goalAngle, -180, 
                180),
                LOWER_ANGLE_LIMIT, UPPER_ANGLE_LIMIT
        );
    }
    /**
     * Figures out the position of the arm in degrees based off pure vertical down
     * @return arm angle in degrees
     */
    public double getArmPos() {
        return armAbsoluteEncoder.getPosition(); 

    }
    
    /**
     * Stops feeding voltage into arm
     */
    public void stopArm() {
        if (armMotor != null) {
            armMotor.set(0);
        }
       
    }

    /**
     * Gets velocity of arm
     * @return rpm of arm
     */
    public double getArmVel() {
        return armAbsoluteEncoder.getVelocity();
    }

    @Override
    public void periodic() {
        
        SmartDashboard.putNumber("feed volts", armFeedVolts);
        SmartDashboard.putNumber("ARM ERROR:", Math.abs(armGoal-getArmPos()));
        //SmartDashboard.putNumber("Arm Angle", getArmPos());
        //SmartDashboard.putNumber("raw arm position", armEncoder.getPosition());
        SmartDashboard.putNumber("Arm Velocity", armAbsoluteEncoder.getVelocity());
        
        // System.out.println("_feedVolts: "+ armFeedVolts);
        // System.out.println("pid: "+armkP+", "+armkI+", "+armkD+" | ff sg: "+armkS+", "+armkG);
        // System.out.println("goal angle:" + armGoal);
        //Killed for now under suspect of murdering arm
        //Case number 0001
        //Victim Arm
        //Prime suspect: this code
        // if (getArmPos() < LOWER_ANGLE_LIMIT) { //if the arm is below this angle limit it is supposed to stop applying voltage
        //     armMotor.set(0);
        //     if (Math.abs(getArmPos()-LOWER_ANGLE_LIMIT) > ARM_ERROR_MARGIN){ //if the arm gets too far from the angle limit it applies a reverse voltage to slow it down
        //         armMotor.set(0.02 * armAbsoluteEncoder.getVelocity() + lowerLimitAdjustmentVoltage);
        //     }
            
        // }
        // if (getArmPos() > UPPER_ANGLE_LIMIT) { //if the arm is above is angle limit then it is supposed to stop applying voltage
            
        //     armMotor.set(0);
        //     if (Math.abs(getArmPos() - UPPER_ANGLE_LIMIT) > ARM_ERROR_MARGIN) { //if the arm gets too far from the upper angle limit then it applies a reverse voltage
        //         armMotor.set(-0.02 * armAbsoluteEncoder.getVelocity() + upperLimitAdjustmentVoltage);
        //     }
            
        // }
            
        }
        
    public void initSendable(SendableBuilder builder){
       super.initSendable(builder); 
       builder.addDoubleProperty("arm kS", () -> KS, (value) -> { KS = value; updateFeedforward(); });
       builder.addDoubleProperty("arm kV", ()-> KV, (value) -> { KV = value; updateFeedforward(); });
       builder.addDoubleProperty("arm kA", ()-> KA, (value) -> { KA = value; updateFeedforward(); } );
       builder.addDoubleProperty("arm kG", ()-> KG, (value) -> { KG = value; updateFeedforward(); } );
       builder.addDoubleProperty("arm kP", () -> KP , (value) -> { KP = value; updateArmPID(); });
       builder.addDoubleProperty("arm kI", () -> KI , (value) -> { KI = value; updateArmPID(); });
       builder.addDoubleProperty("arm kD", () -> KD, (value) -> { KD = value; updateArmPID(); });
       builder.addDoubleProperty("Arm Upper Voltage Counter limit", () -> upperLimitAdjustmentVoltage, (value) -> { upperLimitAdjustmentVoltage = value;});
       builder.addDoubleProperty("Arm Lower Voltage Counter limit", () -> lowerLimitAdjustmentVoltage, (value) -> { lowerLimitAdjustmentVoltage = value;});
       builder.addDoubleProperty("Set Arm Max Velocity", () -> armMaxVelocityDegreesPerSecond, (value) -> { armMaxVelocityDegreesPerSecond = value; updateArmPID(); });
       builder.addDoubleProperty("arm angle (degrees)", () -> getArmPos(), null);
       builder.addDoubleProperty("arm output volts", () -> armMotor.getAppliedOutput()*armMotor.getBusVoltage(), null);
       //builder.addDoubleProperty("Set arm Goal Angle in Degrees", () -> armGoal, (value) -> {setArmTrapPosition(value); });
       builder.addDoubleProperty("Set arm position", () -> armGoal, (value) -> {armGoal = value; setArmPosition_2(value); });
    }

    public void moveArm(double speed){
        armMotor.set(speed);
    }

    //Manual SysId-----------------------------------------------------------------------------------------------------------------------------------------

    //remove all other print statements to make this easy to analyze!!!!
    // In order to get values plot the data and do linear regression to find the values

    public void startManualSysID() { //initializes test by starting timers
        
        voltsApplied = 0;
        
        timer.reset();
        timer.start();
        timer2.reset();
        timer2.start();
        System.out.println("**Use these values to plot data and use linear regression to find feedforward values**");
        System.out.println("(ManualSysID Data)------------------------------------------------------------------------------------------------");
        
    }

    public void runFeedforwardTestDynamic(double voltsIncreaseRate, double endTime, double endVolts) {
        
        
        //only run this if you are not going to run anything else
        //while (timer.get() < endTime && voltsApplied < endVolts && getArmPos() > LOWER_ANGLE_LIMIT && getArmPos() < UPPER_ANGLE_LIMIT) { //stops after any of these conditions are met
            
            // Hold each voltage for 2 seconds
            if (timer2.get() >= TEST_DURATION) {
            
            timer2.reset();
            voltsApplied += voltsIncreaseRate;
            System.out.println("| (DYNAMIC) Arm Velocity (rad/s):" + Units.degreesToRadians(getArmVel()) + "| Voltage:" + voltsApplied + "| Arm Angle (radians):" + Math.cos(Units.degreesToRadians(getArmPos())));
            
            }
            // Apply voltage
            armMotor.setVoltage(voltsApplied);
        

        
        // System.out.println("(End of data)---------------------------------------------------------------------------------------------------------------");
        // return; // Test complete

        
    }

    public boolean isManualSysIDTestFinished(){
        return timer.get() > 15 
        || voltsApplied > 1.5 
        || getArmPos() < LOWER_ANGLE_LIMIT 
        || getArmPos() > UPPER_ANGLE_LIMIT;
    }

    public void endManualSysIDTest(){
        armMotor.setVoltage(0);
        timer.stop();
        System.out.println("(End of data)---------------------------------------------------------------------------------------------------------------");
        return; // Test complete
    }

    public void runFeedforwardTestQuasistatic(double voltsIncreaseRate, double secondsPerIncrease, double endTime, double endVolts) {
        startManualSysID();
        //System.out.println("method enabled");
        
        //Only run this if you aren't going to run anything else
        while (timer.get() < endTime && voltsApplied < endVolts && getArmPos() > LOWER_ANGLE_LIMIT && getArmPos() < UPPER_ANGLE_LIMIT) { //stops after any of these conditions are met
            //System.out.println("while loop working");
            if (timer2.get() >= secondsPerIncrease) { //after every defined period it increases the volts
            
                timer2.reset();
                voltsApplied += voltsIncreaseRate;
                //System.out.println("(DYNAMIC) | Voltage:" + voltsApplied + "| Arm Angle (radians):" + Units.degreesToRadians(getArmPos()) + "| Arm Velocity (rad/s):" + Units.degreesToRadians(getArmVel()));
                System.out.println("| (DYNAMIC) Arm Velocity (rad/s):" + Units.degreesToRadians(getArmVel()) + "| Voltage:" + voltsApplied + "| Arm Angle (radians):" + Math.cos(Units.degreesToRadians(getArmPos())));
            }
            // Apply voltage
            armMotor.setVoltage(voltsApplied);
        }

        armMotor.setVoltage(0);
        timer.stop();
        System.out.println("(End of data)---------------------------------------------------------------------------------------------------------------");
        return; // Test complete
    }

    
    
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
       

}
