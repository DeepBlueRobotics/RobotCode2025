package org.carlmontrobotics.subsystems;


import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;


import static org.carlmontrobotics.RobotContainer.*;

import org.carlmontrobotics.Constants;
import org.carlmontrobotics.RobotContainer;

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
import edu.wpi.first.wpilibj.Encoder;

import static org.carlmontrobotics.Constants.AlgaeEffectorc.*;
import static org.carlmontrobotics.Constants.*;


public class AlgaeEffector extends SubsystemBase {

    //motors
    private final SparkFlex topMotor = new SparkFlex(UPPER_MOTOR_PORT, MotorType.kBrushless);
    private final SparkFlex bottomMotor = new SparkFlex(LOWER_MOTOR_PORT, MotorType.kBrushless); 
    private final SparkFlex pincherMotor = new SparkFlex(PINCH_MOTOR_PORT, MotorType.kBrushless);
    private final SparkMax armMotor = new SparkMax(ARM_MOTOR_PORT, MotorType.kBrushless);

    private SparkFlexConfig pincherMotorConfig = new SparkFlexConfig();
    private SparkFlexConfig bottomMotorConfig = new SparkFlexConfig();
    private SparkFlexConfig topMotorConfig = new SparkFlexConfig();
    private SparkMaxConfig  armMotorConfig = new SparkMaxConfig();
    
    private final RelativeEncoder topEncoder = topMotor.getEncoder();
    private final RelativeEncoder bottomEncoder = bottomMotor.getEncoder();
    private final RelativeEncoder pincherEncoder = pincherMotor.getEncoder();
    private final RelativeEncoder armEncoder = armMotor.getEncoder();
    private final RelativeEncoder armAbsoluteEncoder = armMotor.getAlternateEncoder();

    private final SparkClosedLoopController pidControllerTop = topMotor.getClosedLoopController();
    private final SparkClosedLoopController pidControllerBottom = bottomMotor.getClosedLoopController();
    private final SparkClosedLoopController pidControllerPincher = pincherMotor.getClosedLoopController();
    private final SparkClosedLoopController pidControllerArm = armMotor.getClosedLoopController();
    
    private final SimpleMotorFeedforward topFeedforward = new SimpleMotorFeedforward(kS[TOP_ARRAY_ORDER], kV[TOP_ARRAY_ORDER], kA[TOP_ARRAY_ORDER]);
    private final SimpleMotorFeedforward bottomFeedforward = new SimpleMotorFeedforward(kS[BOTTOM_ARRAY_ORDER], kV[BOTTOM_ARRAY_ORDER], kA[BOTTOM_ARRAY_ORDER]);
    private final SimpleMotorFeedforward pincherFeedforward = new SimpleMotorFeedforward(kS[PINCHER_ARRAY_ORDER], kV[PINCHER_ARRAY_ORDER], kA[PINCHER_ARRAY_ORDER]);
    private final SimpleMotorFeedforward armFeedforward = new SimpleMotorFeedforward(kS[ARM_ARRAY_ORDER], kV[ARM_ARRAY_ORDER], kA[ARM_ARRAY_ORDER]);
    //feedforward for arm was added



    //Arm Trapezoid Profile
    private TrapezoidProfile armTrapProfile;
    private TrapezoidProfile.State armGoalState = new TrapezoidProfile.State(0,0); //position,velocity (0,0)


    private double armGoalAngle = 0;

    //--------------------------------------------------------------------------------------------
    public AlgaeEffector() {
        configureMotors();

    }
    //----------------------------------------------------------------------------------------

    private void configureMotors () {
        topMotorConfig.closedLoop.pid(
            Constants.kP[TOP_ARRAY_ORDER],
            Constants.kI[TOP_ARRAY_ORDER],
            Constants.kD[TOP_ARRAY_ORDER]
            ).feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        topMotor.configure(topMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        
        bottomMotorConfig.closedLoop.pid(
            Constants.kP[BOTTOM_ARRAY_ORDER],
            Constants.kI[BOTTOM_ARRAY_ORDER],
            Constants.kD[BOTTOM_ARRAY_ORDER]
            ).feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        bottomMotor.configure(bottomMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);   
    
        pincherMotorConfig.closedLoop.pid(
            Constants.kP[PINCHER_ARRAY_ORDER],
            Constants.kI[PINCHER_ARRAY_ORDER],
            Constants.kD[PINCHER_ARRAY_ORDER]
            ).feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        pincherMotor.configure(pincherMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        armMotorConfig.closedLoop.pid(
            Constants.kP[ARM_ARRAY_ORDER],
            Constants.kI[ARM_ARRAY_ORDER],
            Constants.kD[ARM_ARRAY_ORDER]
            ).feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        pincherMotor.configure(pincherMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        armMotorConfig.idleMode(IdleMode.kBrake);
        armMotorConfig.closedLoop.pid(
            Constants.kP[ARM_ARRAY_ORDER],
            Constants.kI[ARM_ARRAY_ORDER],
            Constants.kD[ARM_ARRAY_ORDER]
            ).feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    }

    public void setTopRPM(double toprpm) {
        pidControllerTop.setReference(toprpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        topFeedforward.calculate(toprpm);
    }

    public void setBottomRPM(double bottomrpm) {
        pidControllerBottom.setReference(bottomrpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        bottomFeedforward.calculate(bottomrpm);
    }

    public void setPincherRPM(double pincherrpm) {
        pidControllerPincher.setReference(pincherrpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        pincherFeedforward.calculate(pincherrpm);    
    }
    //arm methods
    public void setArmPosition(double armangle) {
        pidControllerArm.setReference(armangle, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        pincherFeedforward.calculate(pincherrpm);    
    }
  
    public void setArmTarget(double targetPost){

    }

    //returns the arm position and velocity based on encoder and position 
    public TrapezoidProfile.State getArmState(){
        TrapezoidProfile.State armState = new TrapezoidProfile.State(getArmPos(), getArmVel());
        return armState;
    }

    public boolean armAtGoal(){
        //TODO:
        //reutrns if arm is at goal state - need to add toelrance
        return false; 
    }

    

    

    public double getArmPos() {
        //figures out the position of the arm in degrees based off pure vertical down
        //TODO update the arm to get in degrees after someone will figure out what the .getPosition gets for the TBE
        return armAbsoluteEncoder.getPosition() * Constants.AlgaeEffectorc.armChainGearing - Constants.AlgaeEffectorc.armToZero;
    }
    public double getArmVel(){
        return armAbsoluteEncoder.getVelocity();
    }

    public void stopArm() {
        armMotor.set(0);

    }


    // public double getArmVel() {
    //     return armAbsoluteEncoder.getVelocity();
    // }

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
        return topEncoder.getVelocity() == rpm;
    }

    public boolean checkIfAtBottomRPM(double rpm) {
        return bottomEncoder.getVelocity() == rpm;
    }

    public void setMotorSpeed(double topSpeed, double bottomSpeed, double pincherSpeed) {
        topMotor.set(topSpeed);
        bottomMotor.set(bottomSpeed);
        pincherMotor.set(pincherSpeed);
    }

    public boolean isAlgaeIntaked() {
        return pincherMotor.getOutputCurrent() > Constants.AlgaeEffectorc.pincherCurrentThreshold;
    }


    


    @Override
    public void periodic() {
        pidControllerArm.setReference(armGoalAngle, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        pincherFeedforward.calculate(goalAngle); //What is this for
    }

}
