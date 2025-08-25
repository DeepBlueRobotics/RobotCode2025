package org.carlmontrobotics.subsystems;

import org.carlmontrobotics.lib199.MotorConfig;
//import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;


import static org.carlmontrobotics.RobotContainer.*;

import java.util.function.BooleanSupplier;

import org.carlmontrobotics.Constants;
import org.carlmontrobotics.RobotContainer;
// import org.carlmontrobotics.commands.AlgaeCommands.ManualDynamicForArm;
// import org.carlmontrobotics.commands.AlgaeCommands.ManualQuasistaticForArm;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.carlmontrobotics.Constants.AlgaeEffectorc.*;


public class AlgaeEffector extends SubsystemBase {

    private final SparkMax armMotor = MotorControllerFactory.createSparkMax(ARM_MOTOR_PORT, MotorConfig.NEO);

    private SparkMaxConfig  armMotorConfig = new SparkMaxConfig();

    private final RelativeEncoder armEncoder = (armMotor != null ? armMotor.getEncoder() : null);
    private final AbsoluteEncoder armAbsoluteEncoder = (armMotor != null ? armMotor.getAbsoluteEncoder() : null);
    

    //--------------------------------------------------------------------------------------------
    public AlgaeEffector() {
        
        configureMotors();
        
        // updateArmPID(); 
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
        armMotorConfig.inverted(true);
        armMotorConfig.absoluteEncoder.zeroOffset(ARM_ZERO_ROT);
        armMotorConfig.absoluteEncoder.zeroCentered(true);
        armMotorConfig.absoluteEncoder.inverted(true);
        armMotorConfig.encoder.positionConversionFactor(ROTATION_TO_DEG * ARM_CHAIN_GEARING);
        armMotorConfig.absoluteEncoder.positionConversionFactor(ROTATION_TO_DEG * ARM_CHAIN_GEARING);
        armMotorConfig.absoluteEncoder.velocityConversionFactor(6 * ARM_CHAIN_GEARING); // 6 is rotations/min to degrees/second

        if (armMotor != null) {
            armMotor.configure(armMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        }
        
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
        SmartDashboard.putNumber("Arm Velocity", armAbsoluteEncoder.getVelocity());  
        SmartDashboard.putNumber("Arm Pos", getArmPos());      
        }
        
    public void initSendable(SendableBuilder builder){
       super.initSendable(builder); 
       builder.addDoubleProperty("arm angle (degrees)", () -> getArmPos(), null);
       builder.addDoubleProperty("arm output volts", () -> armMotor.getAppliedOutput()*armMotor.getBusVoltage(), null);
    }

    public void moveArm(double speed){
        armMotor.set(speed);
    }  

}
