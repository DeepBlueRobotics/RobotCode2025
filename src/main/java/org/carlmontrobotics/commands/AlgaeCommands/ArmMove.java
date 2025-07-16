package org.carlmontrobotics.commands.AlgaeCommands;

import org.carlmontrobotics.subsystems.AlgaeEffector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class ArmMove extends Command{
    AlgaeEffector algaeArm;
    Timer timer = new Timer();
    private double speed;

    public ArmMove(AlgaeEffector algaeArm, double speed){
        addRequirements(this.algaeArm = algaeArm);
        this.speed = speed;
    }

    @Override
    public void initialize(){
        timer.restart();
        algaeArm.moveArm(speed); 

    }

    @Override
    public void execute(){}

    @Override
    public void end(boolean interrupted){
        if (speed > 0) { // This if statement is checking when arm moves up
          algaeArm.moveArm(0.1);  // Small voltage to hold arm up like a kG 
        } else { // This one is for the arm moving down
         algaeArm.moveArm(0);
        }
    }
    
    public boolean isFinished(){
        return (algaeArm.getArmPos() < -10 && speed < 0); // If arm is going down past delagification angle it shuts the voltage off to stop it from hitting itself on the way down too hard
    }
}
