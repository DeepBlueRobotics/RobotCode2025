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
import org.carlmontrobotics.Constants.AlgaeEffectorc;
import org.carlmontrobotics.RobotContainer;
import org.carlmontrobotics.commands.DealgaficationIntake;
import org.carlmontrobotics.commands.GroundIntakeAlgae;
import org.carlmontrobotics.commands.ManualDynamicForArm;
import org.carlmontrobotics.commands.ManualQuasistaticForArm;
import org.carlmontrobotics.commands.OuttakeAlgae;


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


//FOR ROBOT SIMULATION:
import edu.wpi.first.math.controller.PIDController; // For PID control
import edu.wpi.first.wpilibj.simulation.PWMSim; // For simulating a motor
import edu.wpi.first.wpilibj.simulation.EncoderSim; // For simulating an encoder
import edu.wpi.first.wpilibj.Encoder; // For encoder hardware abstraction


import static org.carlmontrobotics.Constants.AlgaeEffectorc.*;
import static org.carlmontrobotics.Constants.*;
import edu.wpi.first.util.sendable.*;


public class AlgaeEffector extends SubsystemBase {

    private double armGoal = LOWER_ANGLE_LIMIT; 
    
    private double clampedArmGoal = LOWER_ANGLE_LIMIT;
    private double armMaxVelocityDegreesPerSecond = 720;
    private final double dT = 0.02; // Time step of 0.02 seconds used for calculations revolving tiny intervals. 
    //This is for the manual sysID methods
    private final Timer timer = new Timer();
    private final Timer timer2 = new Timer();
    private double TEST_DURATION = 2.0;
    private double voltsApplied = 0;

    private double upperLimitAdjustmentVoltage = -0.2;
    private double lowerLimitAdjustmentVoltage = 0.2;
    private double armFeedVolts;
    private double armGoalVelocity = 0;

    /*currently lib199's code for robot simulation does not work. When you use motorcontrollerfactory.createsparkmax() it will create
     a simulated sparkmax instead if there is no actual robot, but since the simulated sparkmax doesn't work then when it does that it will result in an error */

    //private final SparkMax armMotor = MotorControllerFactory.createSparkMax(ARM_MOTOR_PORT, MotorConfig.NEO);
    private final SparkMax armMotor = new SparkMax(ARM_MOTOR_PORT, MotorType.kBrushless);

    private SparkMaxConfig  armMotorConfig = new SparkMaxConfig();

    private final RelativeEncoder armEncoder = armMotor.getEncoder();
    private final AbsoluteEncoder armAbsoluteEncoder = armMotor.getAbsoluteEncoder();

    
    private final SparkClosedLoopController pidControllerArm = armMotor.getClosedLoopController();
    AbsoluteEncoderConfig config = new AbsoluteEncoderConfig();

    //this creates new variables for PID and FF to use in the initsendable. The initial values of these will be set to the current values in constants.java
    private double KS = armKS;
    private double KV = armKV;
    private double KA = armKA;
    private double KG = armKG;
    private double KP = armKP;
    private double KI = armKI;
    private double KD = armKD;
    private ArmFeedforward armFeedforward = new ArmFeedforward(KS, KG, KV, KA);
    private void updateFeedforward() {
        armFeedforward = new ArmFeedforward(KS, KG, KV, KA);
        //System.out.println("kG" + armkG+"*********************");
    }

    //FOR ROBOT SIMULATION:
    private final PWMSim motorSim = new PWMSim(0); // Simulated motor on PWM channel 0
    private final Encoder encoder = new Encoder(1, 2); // Encoder on channels 1 and 2
    private final EncoderSim encoderSim = new EncoderSim(encoder); // Simulated encoder
    private PIDController pidController = new PIDController(KP, KI, KD); // PID controller

    private double velocitySIM;
    private double positionIncrementSIM;
    private double positionSIM;
    private double pidOutput;
    private double rawPIDOutput;
    private double velocityCommand;
    private double previousPositionSIM = LOWER_ANGLE_LIMIT;
    private double positionDerivative;
    private double weightFactor = 0; //this is used to simulate the weight of the arm. This is not measured in any units
    private double weightEffect = 0; //This is used to simulate the effect of the weight of the arm (not 100% realistic)
    private TrapezoidProfile.State setPointSIM = new TrapezoidProfile.State(getSimulatedPosition(), getSimulatedVelocity());

    
    
  
    
    private void updateArmPID() {
        //Update arm PID values for simulation
        pidController = new PIDController(KP, KI, KD);

        // Update the arm motor PID configuration with the new values
        armMotorConfig.closedLoop.pid(KP, KI , KD).maxMotion.maxVelocity(armMaxVelocityDegreesPerSecond).maxAcceleration(100);
        armMotor.configure(armMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
       
    }
    //TrapezoidProfile components
    
    
    private TrapezoidProfile.Constraints armTrapConstraints = new TrapezoidProfile.Constraints(armMaxVelocityDegreesPerSecond, 100); 
    private TrapezoidProfile.State armSetPoint = new TrapezoidProfile.State(getArmPos(), getArmVel()); //this object is used for the current state of the arm and also the calculated state in dT seconds
    private TrapezoidProfile.State goalState = new TrapezoidProfile.State(armGoal, armGoalVelocity); //this is also used for the simulation
    private TrapezoidProfile armTrapProfile = new TrapezoidProfile(armTrapConstraints);
    
    
    

    //--------------------------------------------------------------------------------------------
    public AlgaeEffector() {
        // encoder.setDistancePerPulse((TBE_DPP * ARM_CHAIN_GEARING)); // Set encoder distance per pulse for simulation
        configureMotors();
        
        
        updateArmPID(); 

        SmartDashboard.putData("Arm to Zero Degrees",new InstantCommand(() -> armToTarget(0)));
        
        SmartDashboard.putData("Arm to Intake Angle",new InstantCommand(() -> armToTarget(Constants.AlgaeEffectorc.ARM_INTAKE_ANGLE)));
        SmartDashboard.putData("Arm to Dealgafication Angle",new InstantCommand(() -> armToTarget(Constants.AlgaeEffectorc.ARM_DEALGAFYING_ANGLE)));
        
       
        SmartDashboard.putData("Dealgafication", new DealgaficationIntake(this));
        SmartDashboard.putData("Intake Algae", new GroundIntakeAlgae(this));
        SmartDashboard.putData("Outtake Algae", new OuttakeAlgae(this));
        SmartDashboard.putData("UPDATE COMMAND",new InstantCommand(()->{updateArmPID();updateFeedforward();}));

        SmartDashboard.putData("(MANUAL) Dynamic FF test", new ManualDynamicForArm(this)); 
        SmartDashboard.putData("(MANUAL) Quasistatic FF test", new ManualQuasistaticForArm(this)); 

        //For TrapezoidProfile:
        setPointSIM = getCurrentStateSim();

    }
    //----------------------------------------------------------------------------------------

    private void configureMotors () { //This sets the settings for the motors and encoders by setting the PID values and other settings
    
        armMotorConfig.closedLoop.pid(
            KP, 
            KI,  
            KD 
            ).feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        
        // armMotorConfig.closedLoop.pid( //TODO: revert to this when you get PID values
        //     armKP,
        //     armKI,
        //     armKD
        //     ).feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        armMotorConfig.idleMode(IdleMode.kBrake);
        armMotorConfig.inverted(true);
        armMotorConfig.absoluteEncoder.zeroOffset(ARM_ZERO_ROT);
        armMotorConfig.absoluteEncoder.zeroCentered(true);
        armMotorConfig.absoluteEncoder.inverted(true);
        armMotorConfig.softLimit.forwardSoftLimit(UPPER_ANGLE_LIMIT);
        armMotorConfig.softLimit.forwardSoftLimitEnabled(true);
        armMotorConfig.softLimit.reverseSoftLimit(LOWER_ANGLE_LIMIT);
        armMotorConfig.softLimit.reverseSoftLimitEnabled(true);
        armMotorConfig.encoder.positionConversionFactor(ROTATION_TO_DEG * ARM_CHAIN_GEARING);
        armMotorConfig.absoluteEncoder.positionConversionFactor(ROTATION_TO_DEG * ARM_CHAIN_GEARING);
        armMotorConfig.absoluteEncoder.velocityConversionFactor(6 * ARM_CHAIN_GEARING); // 6 is rotations/min to degrees/second

        if (armMotor != null) {
            armMotor.configure(armMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        }
        
    }
    
    public void setGoalPosition(double targetPos) {
        armGoal = targetPos;
        clampedArmGoal = getArmClampedGoal(armGoal);
        goalState = new TrapezoidProfile.State(clampedArmGoal, armGoalVelocity); //this is for trapezoidProfile
    }
    public void armToTarget(double targetPos){ //this method takes in an angle and sets the arm to that angle 

         setGoalPosition(targetPos);

        if (armMotor != null) {
            
            armFeedVolts = armFeedforward.calculate(Units.degreesToRadians(clampedArmGoal), 0.00001*(armGoal- getArmPos()));//this calculates the amount of voltage needed to move the arm
            pidControllerArm.setReference(clampedArmGoal, ControlType.kPosition, ClosedLoopSlot.kSlot0, armFeedVolts); //This moves the arm to the goal angle and uses PID 

        }
        
    }
    //move arm to position using TrapezoidProfile
    public void setArmTrapPosition(double targetPos){ //run this method in periodic()
        
        
        armSetPoint = armTrapProfile.calculate(dT, armSetPoint, goalState); //this updates the current position of the arm in accordance to trapezoidal motion

        armFeedVolts = armFeedforward.calculate(Units.degreesToRadians(armSetPoint.position), Units.degreesToRadians(armSetPoint.velocity));//this calculates the amount of voltage needed to move the arm for the calculated position in dT seconds
        pidControllerArm.setReference(armSetPoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, armFeedVolts); //This moves the arm to the goal angle and uses PID 
    }
    
    public boolean armAtGoal(){ //This method returns if the arm is at its goal position with the error margin as the tolerance range
        return Math.abs(getArmPos()-armGoal) <= ARM_ERROR_MARGIN;
    }

    public double getArmClampedGoal(double goalAngle) {
        //this method takes in the input angle and sets it to the range given(Lower angle limit to upper angle limit) 
        return MathUtil.clamp(
            MathUtil.inputModulus(goalAngle, -180, 
                180),
                LOWER_ANGLE_LIMIT, UPPER_ANGLE_LIMIT
        );
    }

    public double getArmPos() {
        //figures out the position of the arm in degrees based off pure vertical down
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

    @Override
    public void periodic() {
        
        SmartDashboard.putNumber("feed volts", armFeedVolts);
        SmartDashboard.putNumber("ARM ERROR:", Math.abs(armGoal-getArmPos()));
        
        SmartDashboard.putNumber("raw arm position", armEncoder.getPosition());
        SmartDashboard.putNumber("Arm Velocity", armAbsoluteEncoder.getVelocity());
        
        // System.out.println("_feedVolts: "+ armFeedVolts);
        // System.out.println("pid: "+armkP+", "+armkI+", "+armkD+" | ff sg: "+armkS+", "+armkG);
        // System.out.println("goal angle:" + armGoal);
        
        if (getArmPos() < LOWER_ANGLE_LIMIT) { //if the arm is below this angle limit it is supposed to stop applying voltage
            armMotor.set(0);
            if (Math.abs(getArmPos()-LOWER_ANGLE_LIMIT) > ARM_ERROR_MARGIN){ //if the arm gets too far from the angle limit it applies a reverse voltage to slow it down
                armMotor.set(0.02 * armAbsoluteEncoder.getVelocity() + lowerLimitAdjustmentVoltage);
            }
            
        }
        if (getArmPos() > UPPER_ANGLE_LIMIT) { //if the arm is above is angle limit then it is supposed to stop applying voltage
            
            armMotor.set(0);
            if (Math.abs(getArmPos() - UPPER_ANGLE_LIMIT) > ARM_ERROR_MARGIN) { //if the arm gets too far from the upper angle limit then it applies a reverse voltage
                armMotor.set(-0.02 * armAbsoluteEncoder.getVelocity() + upperLimitAdjustmentVoltage);
            }
            
        }
            
        }

        //FOR ROBOT SIMULATION-----------------------------------------

        //For the simulation we need to manually update the encoders 

        public TrapezoidProfile.State getCurrentStateSim() {
            // Return the next state of the trapezoid profile
            return new TrapezoidProfile.State(getSimulatedPosition(), getSimulatedVelocity());
        }

        public double getSimulatedPosition() {
            // Return the simulated position in degrees
            return positionSIM; // Already in degrees
        }
        
        public double getSimulatedVelocity() {
            // Return the simulated velocity in degrees per second
            return velocitySIM; // Already in degrees per second
        }

        

        
        
        public void setSimulatedPosition(double targetPos) {
            // goalCheck = targetPos; // Update the previous goal angle
            // if (clampedArmGoal != goalCheck){
            //     dTsim = 0.1; // Reset dTsim if the goal changes
            // }
             // Clamp the goal angle between -180 and 180 degrees
            
            //goalCheck = clampedArmGoal; // Update the previous goal angle
            setPointSIM = armTrapProfile.calculate(dT, setPointSIM, goalState); // Calculate the next state of the trapezoid profile
            
        
            // Scale PID output to match the motor's input range (-1 to 1) and cap velocity at max
            rawPIDOutput = pidController.calculate(getSimulatedPosition(), setPointSIM.position);
            velocityCommand = MathUtil.clamp(rawPIDOutput, -armMaxVelocityDegreesPerSecond, armMaxVelocityDegreesPerSecond); // Cap velocity at max (720 degrees/second)
            pidOutput = MathUtil.clamp(velocityCommand / armMaxVelocityDegreesPerSecond, -1, 1); // Scale to motor input range (-1 to 1)
            motorSim.setSpeed(pidOutput); // Set the simulated motor speed

            // if ( Math.abs(clampedArmGoal - setPointSIM.position) <= 2.0 ){
            //     dTsim = 0.1; // If the arm is at the goal, reset dTsim
            // }
            // else if (Math.abs(nextStateSIM.position - setPointSIM.position) <= 1) {
            //     dTsim += 0.1; //if it has reached its mini goal then it will increase dTsim
            // }
        }
        
        @Override
        public void simulationPeriodic() {
            // Update the simulated velocity and position
            weightEffect = weightFactor * 10 * Math.cos(Units.degreesToRadians(getSimulatedPosition())); // Calculate the weight effect based on the current position
            weightEffect = MathUtil.clamp(weightEffect, -0.25 * armMaxVelocityDegreesPerSecond, 0.25 * armMaxVelocityDegreesPerSecond); // Clamp the weight effect to a reasonable range
            velocitySIM = motorSim.getSpeed() * armMaxVelocityDegreesPerSecond - weightEffect; // Convert motor speed to degrees per second (max speed = 720 degrees/second)
            

            encoderSim.setRate(velocitySIM); // Set the simulated encoder rate
        
            positionIncrementSIM = velocitySIM * dT; // Calculate the distance moved over dT time period (d = vt)
            positionSIM = MathUtil.clamp(MathUtil.inputModulus(encoderSim.getDistance() + positionIncrementSIM, -180, 180),
             LOWER_ANGLE_LIMIT, 
             UPPER_ANGLE_LIMIT); // Update the simulated position
            encoderSim.setDistance(positionSIM); // Set the simulated encoder position
            positionDerivative = (positionSIM - previousPositionSIM) / dT; // Calculate the derivative of the position (velocity)
        
            setSimulatedPosition(armGoal); // Update the simulated motor speed based on the PID output
            
        
            // Display simulated values on SmartDashboard
            SmartDashboard.putNumber("(SIMULATED) Current Position", getSimulatedPosition());
            // SmartDashboard.putNumber("(SIMULATED) Next Position", nextStateSIM.position);
            // SmartDashboard.putNumber("(SIMULATED) Next Velocity", nextStateSIM.velocity);
            SmartDashboard.putNumber("(SIMULATED) Current Velocity", velocitySIM);
            SmartDashboard.putNumber("(SIMULATED) PID Output", pidOutput);
            SmartDashboard.putNumber("(SIMULATED) Current Velocity v2", positionDerivative);
            SmartDashboard.putNumber("(SIMULATED) Raw PID Output", rawPIDOutput);
            SmartDashboard.putNumber("(SIMULATED) Velocity Command", velocityCommand);
            SmartDashboard.putNumber("(SIMULATED) Weight Effect", weightEffect);
            previousPositionSIM = positionSIM; // Update the previous position for the next iteration
            

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
       builder.addDoubleProperty("Upper Voltage Counter limit", () -> upperLimitAdjustmentVoltage, (value) -> { upperLimitAdjustmentVoltage = value;});
       builder.addDoubleProperty("Lower Voltage Counter limit", () -> lowerLimitAdjustmentVoltage, (value) -> { lowerLimitAdjustmentVoltage = value;});
       builder.addDoubleProperty("Set Arm Max Velocity", () -> armMaxVelocityDegreesPerSecond, (value) -> { armMaxVelocityDegreesPerSecond = value; updateArmPID(); });
       builder.addDoubleProperty("arm angle (degrees)", () -> getArmPos(), null);
       builder.addDoubleProperty("output volts", () -> armMotor.getAppliedOutput()*armMotor.getBusVoltage(), null);
       builder.addDoubleProperty("Set Goal Angle in Degrees", () -> armGoal, (value) -> {armToTarget(value); });
       builder.addDoubleProperty("(SIMULATION) Weight Factor", () -> weightFactor, (value) -> {weightFactor = value;});
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