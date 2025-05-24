// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.Limelightc.REEF_LL;

import java.util.List;

import org.carlmontrobotics.Constants;
import org.carlmontrobotics.subsystems.Drivetrain;
import org.carlmontrobotics.subsystems.Limelight;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class PathPlannerAlign extends Command {
  private Drivetrain dt;
  private Limelight ll;
  public PathPlannerAlign(Drivetrain drivetrain, Limelight limelight) {
    addRequirements(dt=drivetrain, ll=limelight);
  }

  @Override
  public void initialize() {
    // NOTE this is currently only a test for the reef with april tag 6
    if (ll.seesTag(REEF_LL)) {
      Pose2d currentPos = ll.getRobotPoseInField(REEF_LL);
      Pose2d targetID6 = new Pose2d(5.478, 2.688, Rotation2d.fromDegrees(120.466));
      PathConstraints constraints = new PathConstraints(1, 1, 1, 1);
  
      PathPlannerPath path = new PathPlannerPath(PathPlannerPath.waypointsFromPoses(List.of(currentPos, targetID6)),
       constraints, 
       null, 
       new GoalEndState(0, Rotation2d.fromDegrees(120.466)));
  
      FollowPathCommand command = new FollowPathCommand(
        path, 
        () -> currentPos, 
        () -> dt.getSpeeds(),  
        (speeds, feedforwards) -> dt.drive(dt.getSwerveStates(speeds)),
        new PPHolonomicDriveController(
          new PIDConstants(5.0, 0.0, 0.0),
          new PIDConstants(5.0, 0.0, 0.0)), 
        Constants.Drivetrainc.Autoc.robotConfig, 
        () -> false
        ); // Not sure if dt should be added here as a requirement (extra parameter) since it is already a requirement in the pathplanneralign command
      command.schedule();
    } 
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
