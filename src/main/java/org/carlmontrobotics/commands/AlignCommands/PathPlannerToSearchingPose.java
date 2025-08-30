// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands.AlignCommands;

import static org.carlmontrobotics.Constants.Limelightc.REEF_LL;
import static org.carlmontrobotics.Constants.AligningCords.*;

import java.lang.reflect.Field;
import java.sql.Driver;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.carlmontrobotics.Constants;
import org.carlmontrobotics.Constants.Drivetrainc;
import org.carlmontrobotics.subsystems.Drivetrain;
import org.carlmontrobotics.subsystems.Elevator;
import org.carlmontrobotics.subsystems.Limelight;
import org.carlmontrobotics.subsystems.LimelightHelpers;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class PathPlannerToSearchingPose extends Command {
  private Drivetrain dt;
  private Limelight ll;
  private boolean searchingState = true;
  private final SwerveDrivePoseEstimator poseEstimator;
  private Field2d targetField;
  private boolean first = false;
  private boolean blueAlliance;
  private final Pose2d[] searchPoses = {ID6_17Search, ID7_18Search, ID8_19Search, ID9_20Search, ID10_21Search, ID11_22Search};
  private final List<Integer> blueIDs = List.of(17,18,19,20,21,22);
  private final List<Integer> redIDs = List.of(8,7,6,11,10,9);//I cannot do a freakin array cause it has no indexing option
  private final Pose2d[] rightPoses = {ID6_17Right, ID7_18Right, ID8_19Right, ID9_20Right, ID10_21Right, ID11_22Right};
  private final Pose2d[] leftPoses = {ID6_17Left, ID7_18Left, ID8_19Left, ID9_20Left, ID10_21Left, ID11_22Left};
  private Pose2d targetLocation;
  private int targetID;
  private boolean rightBranch;
  private DoubleSupplier xStick;
  private DoubleSupplier yStick;
  private DoubleSupplier rStick;
  private Command currentPath; 
  private PathConstraints constraints = Drivetrainc.Autoc.pathConstraints; //TODO tune this for fastest possible alignment
  Timer timer;
  private Command moveToAlign;
  private PathPlannerTrajectory currentTrajectory;
  private boolean pathCompleted;
  private XboxController driveRumble;
  Timer pathTimer;

  public PathPlannerToSearchingPose(Drivetrain dt, Elevator elevator, XboxController driveRumble, boolean rightBranch, Limelight limelight,
    DoubleSupplier xStick, DoubleSupplier yStick, DoubleSupplier rStick //For cancellation purposes
    ) {
    this.dt = dt;
    addRequirements(ll=limelight);
    this.driveRumble = driveRumble;
    this.rightBranch = rightBranch;
    moveToAlign = new MoveToAlignReef(dt, ll, elevator, rightBranch, driveRumble);
    poseEstimator = dt.getPoseEstimator();
   
    this.xStick = xStick;
    this.yStick = yStick;
    this.rStick = rStick;
    targetID = -1;
    targetField = new Field2d();
    SmartDashboard.putBoolean("target location is null",targetLocation == null);
    System.out.println(targetLocation);
    System.out.println(targetLocation == null);
    timer = new Timer();
    pathTimer = new Timer();

  }

  @Override
  public void initialize() {
    SmartDashboard.putData("targetField", targetField);
    SmartDashboard.putBoolean("target location is null",targetLocation == null);
    SmartDashboard.putBoolean("searchisyes", searchingState);
    setIfBlueAlliance(); 
    timer.restart();
    goToClosestReef_NoLimelightNeeded();
  }
  
    private void checkCompletionOfPath() {
    Pose2d endPose = currentTrajectory.getEndState().pose;
    Pose2d currentPose = poseEstimator.getEstimatedPosition();
    Translation2d delta = endPose.getTranslation().minus(currentPose.getTranslation());
    double distanceToGoal = delta.getNorm(); // meters

    // Rotation difference
    double angleDifference = endPose.getRotation().minus(currentPose.getRotation()).getRadians();
    double positionTolerance = 0.05; // meters
    double angleTolerance = 0.05; // radians (~3 degrees)

    pathCompleted = (distanceToGoal < positionTolerance) && (Math.abs(angleDifference) < angleTolerance);
  }
  
  private void followTrajectoryManually() {
    double t = pathTimer.get(); // time since starting path
    PathPlannerTrajectoryState state = currentTrajectory.sample(t);
    Pose2d currentPose = poseEstimator.getEstimatedPosition();
    Pose2d errorPose = currentPose.relativeTo(state.pose);
    Translation2d error = errorPose.getTranslation();

    double heading = currentPose.getRotation().getRadians();
    double forward =  Math.cos(heading) * error.getX() + Math.sin(heading) * error.getY();
    double left    = -Math.sin(heading) * error.getX() + Math.cos(heading) * error.getY();

    forward *= 4.0; //KP
    left    *= 4.0;//KP

    double rotation = (state.heading.getRadians() - currentPose.getRotation().getRadians()) * 1; //kP

    dt.drive(forward, left, rotation);
  }

  @Override
  public void execute() {
    SmartDashboard.putBoolean("target location is null",targetLocation == null);
    SmartDashboard.putBoolean("searching state", searchingState);
    // if (searchingState && ll.seesTag(REEF_LL) && (int) LimelightHelpers.getFiducialID(REEF_LL) != targetID) {
    //   currentPath.cancel();
    //   runPathToClosestReef();
    //   searchingState = false;
    if (first == false) {
      
      first = true;
    }
    

    // }
    // else if (searchingState && ll.seesTag(REEF_LL)) {
    //   searchingState = false; // what does this do? Turns off searching state so that the other function wont turn on
    // }
    // if (!searchingState && !ll.seesTag(REEF_LL)) {
    //   currentPath.cancel();
    //   runToClosestSearchingPosition();
    //   searchingState = true;
    // }
    //I feel like the problem with this thing is that its gonna activate if the tag is flickering which is bad
    SmartDashboard.putData("targetField", targetField);
    System.out.println(targetLocation);
    System.out.println(targetLocation == null);
  }

  @Override
  public void end(boolean interrupted) {
    if (currentPath!=null) {
      currentPath.cancel();
    }
    moveToAlign.schedule();
  }

  @Override
  public boolean isFinished() {
    return currentPath == null || (
      currentPath.isFinished()) || 
      (Math.abs(xStick.getAsDouble()) > 0.1 ) || 
      (Math.abs(yStick.getAsDouble()) > 0.1 ) || 
      (Math.abs(rStick.getAsDouble()) > 0.1 ) ||
      timer.get() > 3;
  }

  private boolean setIfBlueAlliance(){
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      blueAlliance = true;
      SmartDashboard.putBoolean("BlueAlliance", true);
    }
    else if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      blueAlliance = false;
      SmartDashboard.putBoolean("BlueAlliance", false);
    }
    return blueAlliance;
  }

  private void goToClosestReef_NoLimelightNeeded(){

    Pose2d currentPose = poseEstimator.getEstimatedPosition();
    Pose2d[] searchingPoses = searchPoses;

    double closestDistance = Double.MAX_VALUE;
    Pose2d cloestPose = null;

    for (Pose2d searchPose : searchingPoses) {

      if (setIfBlueAlliance() == false || currentPose.getX() > 10){ //when the robot's x coordinate is greater than 10 then it should not try to move to its own reef (10 is just a rough estimate)
        //If the alliance is red or the robot is on the opposite side of the field then it will flip the coordinates so that it looks at the reef coordinates on the opposite side
        searchPose = FlippingUtil.flipFieldPose(searchPose); 
      }

      //get the distance from the current pose to the branch pose
      double distance = currentPose.getTranslation().getDistance(searchPose.getTranslation());
      //if the distance is less than any of the other distances checked it will set the closest branch to that pose
      if (distance < closestDistance) {
        closestDistance = distance;
        cloestPose = searchPose;
      }
    }

    targetLocation = cloestPose;
    targetField.setRobotPose(targetLocation);

    PathPlannerPath path = new PathPlannerPath(PathPlannerPath.waypointsFromPoses(
      List.of(poseEstimator.getEstimatedPosition(), targetLocation)),
      constraints, 
      null, 
      new GoalEndState(1.0, targetLocation.getRotation()));
    path.preventFlipping = true;
    
    currentPath = AutoBuilder.followPath(path); 
    currentPath.schedule();

  }
  public boolean maneImCloseEnoughToTheGoalStateForPathPlanner() {
    if (targetLocation != null) {
      double xDist = Math.abs(poseEstimator.getEstimatedPosition().getX() - targetLocation.getX());
      double yDist = Math.abs(poseEstimator.getEstimatedPosition().getY() - targetLocation.getY());
      double rotDist = Math.abs(poseEstimator.getEstimatedPosition().getRotation().getDegrees() - targetLocation.getRotation().getDegrees());
      if (xDist < 0.1 && yDist < 0.1 && rotDist < 2) {
        return true;
      }
    }
    return false;
  }
}
