package org.carlmontrobotics.commands;

import org.carlmontrobotics.subsystems.AlgaeEffector;

import edu.wpi.first.wpilibj2.command.Command;


public class ArmToPosition extends Command{
    AlgaeEffector algaeArm;
    private double goalPosition; //radians - need angle for dealgfication and ground pickup
   

    public ArmToPosition(AlgaeEffector algaeArm, double goalPosition){
        addRequirements(this.algaeArm = algaeArm);
        this.goalPosition = goalPosition;
       
    }

    @Override
    public void initialize(){
        //algaeArm.setArmTarget(goalPosition); //sets target to position in rads
    }

    // @Override
    // public void execute(){}

    // @Override
    // public void end(){boolean interrupted}
    
    public boolean isFinished(){
        return false;//rreturn algaeArm.armAtGoal();
    }
}
