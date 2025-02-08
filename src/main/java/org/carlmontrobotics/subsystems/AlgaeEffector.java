package org.carlmontrobotics.subsystems;


import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;


import static org.carlmontrobotics.RobotContainer.*;

import org.carlmontrobotics.Constants;
import org.carlmontrobotics.RobotContainer;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkMax;

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


public class AlgaeEffector extends SubsystemBase {
    private SparkFlex topMotor = new SparkFlex(Constants.CoralEffectorc.effectorMotorID, MotorType.kBrushless); //why is there a vortex motor on coral effectors?
    private SparkMax bottomMotor = new SparkMax(Constants.CoralEffectorc.effectorMotorID, MotorType.kBrushless); //why is there a vortex motor on coral effectors?
    private SparkFlex pincherMotor = new SparkFlex(Constants.CoralEffectorc.effectorMotorID, MotorType.kBrushless); //why is there a vortex motor on coral effectors?
    
    private final RelativeEncoder topEncoder = topMotor.getEncoder();
    private final RelativeEncoder bottomEncoder = bottomMotor.getEncoder();
    private final RelativeEncoder pincherEncoder = pincherMotor.getEncoder();

    private final SparkClosedLoopController pidControllerTop = topMotor.getClosedLoopController();
    private final SparkClosedLoopController pidControllerBottom = bottomMotor.getClosedLoopController();
    private final SparkClosedLoopController pidControllerPincher = pincherMotor.getClosedLoopController();
    
    private final SimpleMotorFeedforward topFeedforward = new SimpleMotorFeedforward(Constants.kS[Constants.top], Constants.kV[Constants.top], Constants.kA[Constants.top]);
    private final SimpleMotorFeedforward bottomFeedforward = new SimpleMotorFeedforward(Constants.kS[Constants.bottom], Constants.kV[Constants.bottom], Constants.kA[Constants.bottom]);
    private final SimpleMotorFeedforward pincherFeedforward = new SimpleMotorFeedforward(Constants.kS[Constants.pincher], Constants.kV[Constants.pincher], Constants.kA[Constants.pincher]);
    //TODO: add feedforward

    DigitalInput limitSwitch = new DigitalInput(1); 
    
    public boolean limitDetects() {
        return limitSwitch.get(); 
    }
    //--------------------------------------------------------------------------------------------
    public AlgaeEffector() {
        SparkFlexConfig topConfig = new SparkFlexConfig();
        SparkFlexConfig bottomConfig = new SparkFlexConfig();
        SparkFlexConfig pincherConfig = new SparkFlexConfig();

        topConfig.closedLoop.pid(
            Constants.kP[Constants.top],
            Constants.kI[Constants.top],
            Constants.kD[Constants.top]
            ).feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        topMotor.configure(topConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        
        bottomConfig.closedLoop.pid(
            Constants.kP[Constants.bottom],
            Constants.kI[Constants.bottom],
            Constants.kD[Constants.bottom]
            ).feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        bottomMotor.configure(bottomConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);   
    
        pincherConfig.closedLoop.pid(
            Constants.kP[Constants.pincher],
            Constants.kI[Constants.pincher],
            Constants.kD[Constants.pincher]
            ).feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        pincherMotor.configure(pincherConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
    //----------------------------------------------------------------------------------------

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

    public void setSpeed(double speed) {
        topMotor.set(speed);
        bottomMotor.set(speed);
        pincherMotor.set(speed);
    }
}
