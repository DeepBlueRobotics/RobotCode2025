
package org.carlmontrobotics.subsystems;
import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static org.carlmontrobotics.Constants.*;

import org.carlmontrobotics.Constants.CoralEffectorConstants;

public class CoralEffector extends SubsystemBase {
  
    public static SparkFlex coralMotor = new SparkFlex(CoralEffectorConstants.coralMotorPort, MotorType.kBrushless);
    public static DigitalInput coralLimitSwitch = new DigitalInput(CoralEffectorConstants.coralLimitSwitchPort);
    public static TimeOfFlight distanceSensor = new TimeOfFlight(CoralEffectorConstants.coralDistanceSensorPort);

    public static boolean distanceSensorSees;
    public static boolean limitSwitchSees;
  
    public CoralEffector(){
      // SparkFlexConfig config = new SparkFexConfig();
      // /*config.closedLoop.*/
      // coralMotor.configure(confi, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

  @Override
  public void periodic() {
     distanceSensorSees = distanceSensor.getRange() < CoralEffectorConstants.coralDistanceSensorDistance;
     limitSwitchSees = !coralLimitSwitch.get();
     SmartDashboard.putBoolean("Distance sensor", distanceSensorSees);
     SmartDashboard.putNumber("distance", distanceSensor.getRange());
     SmartDashboard.putBoolean("limit switch", limitSwitchSees);
  }
}