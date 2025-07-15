package org.carlmontrobotics.commands.AlgaeCommands;

import org.carlmontrobotics.subsystems.AlgaeEffector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class ArmMove extends Command{
    AlgaeEffector algaeArm;
    private double goalPosition; //degrees - need angle for dealgfication and ground pickup
    Timer timer = new Timer();
   

    public ArmMove(AlgaeEffector algaeArm){
        addRequirements(this.algaeArm = algaeArm);
       
    }

    @Override
    public void initialize(){
        timer.restart();
        algaeArm.moveArm(-1); //sets target to position in degrees

    }

    @Override
    public void execute(){}

    @Override
    public void end(boolean interrupted){
        
        algaeArm.moveArm(0);
    }
    
    public boolean isFinished(){
        return timer.get() > 2;
    }
}
