package org.carlmontrobotics.commands.AlgaeCommands;

import static org.carlmontrobotics.Constants.AlgaeEffectorc.LOWER_ANGLE_LIMIT;
import static org.carlmontrobotics.Constants.AlgaeEffectorc.UPPER_ANGLE_LIMIT;

import org.carlmontrobotics.subsystems.AlgaeEffector;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;


public class ManualDynamicForArm extends Command{

    private static final double TEST_DURATION = 2.0; // seconds per voltage
    double voltsApplied;
    AlgaeEffector algaeArm;

    private final Timer timer = new Timer(); // timer that runs when MonkeySysId starts
    private final Timer timer2 = new Timer(); // lap timer

    private double goalPosition; //degrees - need angle for dealgfication and ground pickup
   

    public ManualDynamicForArm(AlgaeEffector algaeArm){
        addRequirements(this.algaeArm = algaeArm);
       
       
    }

    @Override
    public void initialize(){
         
        // voltsApplied = 0;
        
        // timer.reset();
        // timer.start();
        // timer2.reset();
        // timer2.start();
        // System.out.println("**Use these values to plot data and use linear regression to find feedforward values**");
        // System.out.println("(ManualSysID Data)------------------------------------------------------------------------------------------------");
        algaeArm.startManualSysID();
    }

     @Override
     public void execute(){
        algaeArm.runFeedforwardTestDynamic(0.05, 15, 1.5);
        //only run this if you are not going to run anything else
         //stops after any of these conditions are met
            
            // Hold each voltage for 2 seconds
        //     if (timer2.get() >= TEST_DURATION) {
            
        //     timer2.reset();
        //     voltsApplied += 0.02;
        //     System.out.println("(DYNAMIC) | Voltage:" + voltsApplied + "| Arm Angle (radians):" + Units.degreesToRadians(getArmPos()) + "| Arm Velocity (rad/s):" + Units.degreesToRadians(getArmVel()));
            
        //     }
        //     // Apply voltage
        //     algaeArm.armMotor.setVoltage(voltsApplied);
        

        // algaeArm.armMotor.setVoltage(0);
        // timer.stop();
        // System.out.println("(End of data)---------------------------------------------------------------------------------------------------------------");
        // return; // Test complete

     }

    @Override
    public void end(boolean interrupted){
        algaeArm.endManualSysIDTest();
        
    }
    
    public boolean isFinished(){
        
        return algaeArm.isManualSysIDTestFinished();
    }
}
