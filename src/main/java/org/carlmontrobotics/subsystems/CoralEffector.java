package org.carlmontrobotics.subsystems;
import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static org.carlmontrobotics.Constants.*;

import static org.carlmontrobotics.Constants.CoralEffectorc.*;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase;

public class CoralEffector extends SubsystemBase {
  
    public SparkFlex coralMotor = new SparkFlex(CORAL_MOTOR_PORT, MotorType.kBrushless);
    // public DigitalInput coralLimitSwitch = new DigitalInput(CORAL_LIMIT_SWITCH_PORT);
    //FIXME ADD THE LIMIT SWITCH!!
    public TimeOfFlight distanceSensor = new TimeOfFlight(CORAL_DISTANCE_SENSOR_PORT);
    
    
    public final RelativeEncoder coralEncoder = coralMotor.getEncoder();
    
    private double coralOutput;
    public boolean coralIn;
    private double targetPos=0;
  
    SparkFlexConfig config = new SparkFlexConfig();
    public CoralEffector(){
    config
        .inverted(false)
        .idleMode(IdleMode.kBrake);
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(KP, KI, KD);
        
    coralMotor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
   }

  public double getEncoderPos() {
    return coralEncoder.getPosition();
  }
  public void setMotorSpeed(double speed) {
    coralMotor.set(speed);
  }
  public void setReferencePosition(double reference) {
    targetPos = reference;
    coralMotor.getClosedLoopController().setReference(reference, SparkBase.ControlType.kPosition);
  }
  public boolean motorAtGoal(){
    return Math.abs(coralEncoder.getPosition()-targetPos) <= CORAL_INTAKE_ERR;
  }

  public boolean distanceSensorSeesCoral(){
    return distanceSensor.getRange() < CORAL_DISTANCE_SENSOR_DISTANCE;
  }

  // public boolean coralIsIn() {
  //   return coralIn;
  // }
  // public void setCoralIn(boolean coralIsInside) {
  //   coralIn = coralIsInside;
  // }

  // public boolean limitSwitchSeesCoral() {
  //   return !coralLimitSwitch.get();
  // }

  @Override
  public void periodic() {
    //coralMotor.getClosedLoopController().setReference(1, ControlType.kVelocity);
    // limitSwitchSees = !coralLimitSwitch.get();
    // CoralGoalRPM = coralEncoder.getVelocity();
    coralOutput = coralMotor.getAppliedOutput();

    SmartDashboard.putBoolean("Distance sensor", distanceSensorSeesCoral());
    SmartDashboard.putNumber("distance", distanceSensor.getRange());
    SmartDashboard.putBoolean("coral in", coralIn);
    // SmartDashboard.putBoolean("limit switch", limitSwitchSees);
    // SmartDashboard.putNumber("Coral goal RPM", CoralGoalRPM);
    SmartDashboard.putNumber("Coral Speed", coralEncoder.getVelocity());
    SmartDashboard.putNumber("coral output", coralOutput);
    SmartDashboard.getNumber("P", KP);
  }
}