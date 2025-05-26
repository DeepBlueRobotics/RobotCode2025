// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.Limelightc.REEF_LL;
import static org.carlmontrobotics.Constants.AligningCords.*;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.carlmontrobotics.Constants;
import org.carlmontrobotics.subsystems.Drivetrain;

import org.carlmontrobotics.subsystems.Limelight;
import org.carlmontrobotics.subsystems.LimelightHelpers;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

public class PathPlannerToReef extends Command {
  private Drivetrain dt;
  private Limelight ll;
  private boolean searchingState = true;
  private final SwerveDrivePoseEstimator poseEstimator;
  


  private final Pose2d[] searchPoses = {ID6_17Search, ID7_18Search, ID8_19Search, ID9_20Search, ID10_21Search, ID11_22Search};
  private final List<Integer> blueIDs = List.of(17,18,19,20,21,22);
  private final List<Integer> redIDs = List.of(6,7,8,9,10,11);//I cannot do a freakin array cause it has no indexing option
  private final Pose2d[] rightPoses = {ID6_17Right, ID7_18Right, ID8_19Right, ID9_20Right, ID10_21Right, ID11_22Right};
  private final Pose2d[] leftPoses = {ID6_17Left, ID7_18Left, ID8_19Left, ID9_20Left, ID10_21Left, ID11_22Left};
  private Pose2d targetLocation;
  private int targetID;
  private Pose2d finalLocation;
  private boolean rightBranch;
  private DoubleSupplier xStick;
  private DoubleSupplier yStick;
  private DoubleSupplier rStick;
  private Command currentPath; 
  private PathConstraints constraints = new PathConstraints(1, 1, 1, 1); //TODO tune this for fastest possible alignment

  public PathPlannerToReef(Drivetrain drivetrain, Limelight limelight, boolean rightBranch,
    DoubleSupplier xStick, DoubleSupplier yStick, DoubleSupplier rStick //For cancellation purposes
    ) {
    addRequirements(dt=drivetrain, ll=limelight);
    this.xStick = xStick;
    this.yStick = yStick;
    this.rStick = rStick;
    this.rightBranch = rightBranch;
    targetID = -1;
    poseEstimator = dt.getPoseEstimator();
  }

  @Override
  public void initialize() {
    if (ll.seesTag(REEF_LL)) {
      runPathToClosestReef();
      searchingState = false;
    } 
    else {
      runToClosestSearchingPosition();
      searchingState = true;
    }
  }

  @Override
  public void execute() {
    if (searchingState && ll.seesTag(REEF_LL) && (int) LimelightHelpers.getFiducialID(REEF_LL) != targetID) {
      currentPath.cancel();
      runPathToClosestReef();
      searchingState = false;

    }
    else if (searchingState && ll.seesTag(REEF_LL)) {
      searchingState = false;
    }
    // if (!searchingState && !ll.seesTag(REEF_LL)) {
    //   currentPath.cancel();
    //   runToClosestSearchingPosition();
    //   searchingState = true;
    // }
    //I feel like the problem with this thing is that its gonna activate if the tag is flickering which is bad
  }

  @Override
  public void end(boolean interrupted) {
    if (searchingState) {
      currentPath.cancel();
    }
  }

  @Override
  public boolean isFinished() {
    return (currentPath.isFinished()) || (Math.abs(xStick.getAsDouble()) > 0.1 ) || (Math.abs(yStick.getAsDouble()) > 0.1 ) || (Math.abs(rStick.getAsDouble()) > 0.1 );
  }

  private void runPathToClosestReef() {
      targetID = (int) LimelightHelpers.getFiducialID(REEF_LL);
      if (rightBranch) {
        if (blueIDs.contains(targetID)) {
          targetLocation = rightPoses[blueIDs.indexOf(targetID)];
        }
        else if (redIDs.contains(targetID)) {
          targetLocation = rightPoses[redIDs.indexOf(targetID)];
        }
      }
      else {
        if (blueIDs.contains(targetID)) {
          targetLocation = leftPoses[blueIDs.indexOf(targetID)];
        }
        else if (redIDs.contains(targetID)) {
          targetLocation = leftPoses[redIDs.indexOf(targetID)];
        }
      }
      PathPlannerPath path = new PathPlannerPath(PathPlannerPath.waypointsFromPoses(
        List.of(poseEstimator.getEstimatedPosition(), targetLocation)), 
        constraints, 
        null, 
        new GoalEndState(0, targetLocation.getRotation()));
  
      // FollowPathCommand command = new FollowPathCommand(
      //   path, 
      //   () -> poseEstimator.getEstimatedPosition(), 
      //   () -> dt.getSpeeds(),  
      //   (speeds, feedforwards) -> dt.drive(dt.getSwerveStates(speeds)),
      //   new PPHolonomicDriveController(
      //     new PIDConstants(5.0, 0.0, 0.0),
      //     new PIDConstants(5.0, 0.0, 0.0)), 
      //   Constants.Drivetrainc.Autoc.robotConfig, 
      //   () -> false
      //   ); // Not sure if dt should be added here as a requirement (extra parameter) since it is already a requirement in the pathplanneralign command
      currentPath = AutoBuilder.followPath(path); //works like this with already built autobuilder
      currentPath.schedule();
  }
  private void runToClosestSearchingPosition() {
    targetLocation = findClosestPose(poseEstimator.getEstimatedPosition(), searchPoses);
    if (rightBranch) {
        if (blueIDs.contains(targetID)) {
          finalLocation = rightPoses[blueIDs.indexOf(targetID)];
        }
        else if (redIDs.contains(targetID)) {
          finalLocation = rightPoses[redIDs.indexOf(targetID)];
        }
      }
    else {
      if (blueIDs.contains(targetID)) {
        finalLocation = leftPoses[blueIDs.indexOf(targetID)];
      }
      else if (redIDs.contains(targetID)) {
        finalLocation = leftPoses[redIDs.indexOf(targetID)];
      }
    }
    PathPlannerPath path = new PathPlannerPath(PathPlannerPath.waypointsFromPoses(List.of(poseEstimator.getEstimatedPosition(), targetLocation, finalLocation)), 
    constraints, 
    null, 
    new GoalEndState(0, finalLocation.getRotation()));
    currentPath = AutoBuilder.followPath(path); //works like this with already built autobuilder
    currentPath.schedule();
  }

  public Pose2d findClosestPose(Pose2d target, Pose2d[] poses) {
    int closestIndex = -1;
    double minDistance = Double.MAX_VALUE;

    for (int i = 0; i < poses.length; i++) {
      Pose2d pose = poses[i];
      double distance = pose.getTranslation().getDistance(target.getTranslation());
  
      if (distance < minDistance) {
          minDistance = distance;
          closestIndex = i;
      }
    }
    if (closestIndex != -1) {
      if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
        targetID = blueIDs.get(closestIndex);
      }
      else if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
        targetID = redIDs.get(closestIndex);
      }
      return poses[closestIndex];
    }
    return null;
  }

}
