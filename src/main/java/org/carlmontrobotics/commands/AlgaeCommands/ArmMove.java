package org.carlmontrobotics.commands.AlgaeCommands;

import org.carlmontrobotics.subsystems.AlgaeEffector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class ArmMove extends Command{
    AlgaeEffector algaeArm;
    private double goalPosition; //degrees - need angle for dealgfication and ground pickup
    Timer timer = new Timer();
    private double speed;

    public ArmMove(AlgaeEffector algaeArm, double speed){
        addRequirements(this.algaeArm = algaeArm);
        this.speed = speed;
    }

    @Override
    public void initialize(){
        timer.restart();
        algaeArm.moveArm(speed); //sets target to position in degrees

    }

    @Override
    public void execute(){}

    @Override
    public void end(boolean interrupted){
        if (speed == 0.125) {
          algaeArm.moveArm(0.05);  
        } else {
         algaeArm.moveArm(0);
        }
    }
    
    public boolean isFinished(){
        return false;
    }
}
