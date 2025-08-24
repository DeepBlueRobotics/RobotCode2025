// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands.AlignCommands;



import static org.carlmontrobotics.Constants.AligningCords.ID10_21Search;
import static org.carlmontrobotics.Constants.AligningCords.ID11_22Search;
import static org.carlmontrobotics.Constants.AligningCords.ID6_17Search;
import static org.carlmontrobotics.Constants.AligningCords.ID7_18Search;
import static org.carlmontrobotics.Constants.AligningCords.ID8_19Search;
import static org.carlmontrobotics.Constants.AligningCords.ID9_20Search;
import static org.carlmontrobotics.Constants.Drivetrainc.positionTolerance;
import static org.carlmontrobotics.Constants.Drivetrainc.thetaPIDController;
import static org.carlmontrobotics.Constants.Drivetrainc.velocityTolerance;
import static org.carlmontrobotics.Constants.Elevatorc.testl4;
import static org.carlmontrobotics.Constants.Elevatorc.testl4RaiseHeight;
import static org.carlmontrobotics.Constants.Limelightc.LEFT_CORAL_BRANCH;
import static org.carlmontrobotics.Constants.Limelightc.REEF_LL;
import static org.carlmontrobotics.Constants.Limelightc.RIGHT_CORAL_BRANCH;
import static org.carlmontrobotics.Constants.Limelightc.areaPercentageGoal;
import static org.carlmontrobotics.Constants.Limelightc.areaTolerance;
import static org.carlmontrobotics.Constants.Limelightc.strafeTolerance;

import java.util.List;

import org.carlmontrobotics.Constants.Drivetrainc;
import org.carlmontrobotics.Constants.Drivetrainc.Autoc;
import org.carlmontrobotics.subsystems.CoralEffector;
import org.carlmontrobotics.subsystems.Drivetrain;
import org.carlmontrobotics.subsystems.Elevator;
import org.carlmontrobotics.subsystems.Limelight;
import org.carlmontrobotics.subsystems.LimelightHelpers;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GlobalLocalAlign extends Command {
  private enum State {
    NULL,
    ELEVATORNOTDOWN,
    GLOBAL,
    ROTATE,
    LOCAL,
    RAISEELEVATOR,
    SCORE,
    COMPLETE;

    public State nextState() {
      return values()[(this.ordinal() + 1) % values().length];
    }
  }
  //Subsystems
  private Drivetrain dt;
  private Limelight ll;
  private Elevator elevator;
  private CoralEffector coralEffector;
  //Params
  private boolean rightStick;
  private double reefLevel;
  private boolean selfScore;
  //execute vars
  boolean scoredCoral;
  int targetID;
  private State currentState;
  private PoseEstimator poseEstimator = dt.getPoseEstimator();
  //private Command currentPath;
  private PathConstraints constraints = Drivetrainc.Autoc.pathConstraints;
  private Timer scoringTimer;
  private Timer scoringTimerFinalL4;
  private PathPlannerTrajectory currentTrajectory;  // the path you want to follow   
  Timer pathTimer = new Timer();
  private boolean pathCompleted = false;
  //alignment vars
  private Timer didntseetime;
  private Timer timeoutTimer;
  private double strafeClamp;
  private double strafeSpeedMultiplier;
  private double forwardClamp;
  private double forwardSpeedMultiplier;
  private double strafeErr;
  private double forwardErr;

  //Teleop bypassing
  private boolean originalFieldOrientation;
  
  
  public final PIDController rotationPID = new PIDController(thetaPIDController[0], thetaPIDController[1],
            thetaPIDController[2]);

  /**
   * GlobalLocalAlign no self scoring
   * @param dt
   * @param ll
   * @param elevator
   * @param rightStick
   */
  public GlobalLocalAlign(Drivetrain dt, Limelight ll, Elevator elevator, CoralEffector coralEffector, boolean rightStick) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.dt = dt, this.elevator = elevator, this.coralEffector = coralEffector);
    this.ll = ll;
    this.rightStick = rightStick;
    selfScore = false;
    strafeClamp = .35;
    forwardClamp = 1.5; 
    strafeSpeedMultiplier = 5;
    forwardSpeedMultiplier = 0.2;
    didntseetime = new Timer();
    timeoutTimer = new Timer();
    scoringTimer = new Timer();
    scoringTimerFinalL4 = new Timer();
  }

  /**
   * GlobalLocal Align self scoring
   * @param dt Drivetrain
   * @param ll Limelight
   * @param elevator Elevator
   * @param rightStick boolean if stick is right or left
   * @param reefLevel reefLevel to score at
   */
  public GlobalLocalAlign(Drivetrain dt, Limelight ll, Elevator elevator, CoralEffector coralEffector, boolean rightStick, double reefLevel) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.dt = dt, this.elevator = elevator, this.coralEffector = coralEffector);
    this.ll = ll;
    this.rightStick = rightStick;
    this.reefLevel = reefLevel;
    selfScore = true;
    strafeClamp = .35;
    forwardClamp = 1.5; 
    strafeSpeedMultiplier = 5;
    forwardSpeedMultiplier = 0.2;
    didntseetime = new Timer();
    timeoutTimer = new Timer();
    scoringTimer = new Timer();
    scoringTimerFinalL4 = new Timer();;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    /*
     * Reset any timers, or states
     * Detect which state to be in, elevatornot down, global, rotate, local, raise elevator, score, or complete
     * Elevatornotdown gets elevator down to be able to use limelight properly 
     * Global gets to search state, gets correct rotation, and then turns into local state
     * Rotate rotates apprx as needed, and then turns into local staet
     * Local uses local alignment, using MoveToAlign logic
     * 
     * To figure out your initial state
     * Check if elevatordown good
     *    Check if see april tag
     *          Check if rotate good
     *              Check if local good
     *                  Check if selfScore
     *                      RaiseElevator State
     *                  Else
     *                      Complete State
     *               Else
     *                    Local state
     *          Else
     *              Rotate state
     *    Else
     *        Global state
     * Else
     *      elevatornotdown state
     */
    resetVars();
    originalFieldOrientation = dt.getFieldOriented();
    dt.setFieldOriented(false);
    currentState = detectInitialState();
    targetID = getNearestReefTag();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
     *  Update state
     *  Check if current state complete
     *     go to next state
     *  Else 
     *     no change
     * 
     * If current state is local and complete check for selfScore
     *     go to RaiseElevator state
     * Else
     *    go to Complete state
     * 
     * Execute current state
     */
    updateState();
    executeState();
    SmartDashboard.putString("AlignmentState", currentState.toString());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    /*
     * Should stop any path from running, any drivetrain from running, and any timers
     */
    dt.setFieldOriented(originalFieldOrientation);
    dt.drive(0,0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*
     * Checks if complete state
     */
    return (currentState == State.COMPLETE);
  }
  /**
   * Resets all variables before each GlobalLocalAlign
   */
  private void resetVars() {
    currentState = State.NULL;
    currentTrajectory = null;
    scoredCoral = false;
    didntseetime.reset();
    didntseetime.stop();
    timeoutTimer.reset();
    timeoutTimer.stop();
    scoringTimer.stop();
    scoringTimer.reset();
    scoringTimerFinalL4.stop();
    scoringTimerFinalL4.reset();
    pathTimer.reset();
    pathTimer.stop();
    strafeErr = Double.POSITIVE_INFINITY;
    forwardErr = Double.POSITIVE_INFINITY;
    targetID = -1;
    pathCompleted = false;
  }

  /**
   * Figures out the intial state the robot is in, can get up to be fully aligned, and ready to score/ complete with task
   * @return a State enum
   */
  private State detectInitialState() {
    /*
     * To figure out your initial state
     * Check if elevatordown good
     *    Check if within local area
     *          Check if rotate good
     *              Check if local good
     *                  Check if selfScore
     *                      RaiseElevator State
     *                  Else
     *                      Complete State
     *               Else
     *                    Local state
     *          Else
     *              Rotate state
     *    Else
     *        Global state
     * Else
     *      elevatornotdown state
     */
    if (elevator.getBottomLimitSwitch()) {
      if (detectWithinLocal()) {
        if (checkRotation()) {
          if (checkLocalAlignment()) {
            if (selfScore) {
              return State.RAISEELEVATOR;
            }
            else {
              return State.COMPLETE;
            }
          }
          else {
            return State.LOCAL;
          }
        }
        else {
          return State.ROTATE;
        }
      }
      else {
        return State.GLOBAL;
      }
    }
    else {
      return State.ELEVATORNOTDOWN;
    }
  }

  private void updateState() {
    /*
     *  Update state
     *  Check if current state complete
     *     go to next state
     *  Else 
     *     no change
     * 
     * If current state is local and complete check for selfScore
     *     go to RaiseElevator state
     * Else
     *    go to Complete state
     */
    int index = currentState.ordinal();
    switch (index) {
      case 0: //Elevator not down
        if (elevator.getBottomLimitSwitch()) {
          currentState = currentState.nextState();
        }
        break;
      case 1: // global
        if (currentTrajectory != null && false) {
          currentState = currentState.nextState();
        }
        break;
      case 2: // rotate
        if (checkRotation()) {
          currentState = currentState.nextState();
        }
        break;
      case 3: // local
        if (checkLocalAlignment()) {
          if (selfScore)
            currentState = currentState.nextState();
          else {
            currentState = State.COMPLETE;
          }
        }
        break;
      case 4: // raise elevator
        if (elevator.getGoal() == reefLevel && elevator.atGoalHeight()) {
          currentState = currentState.nextState();
        }
        break;
      case 5: //score
        if (scoredCoral) {
          currentState = currentState.nextState();
        }
        break;
      default:
        break;
    }

  }

  private void executeState() {
    int index = currentState.ordinal();
    switch (index) {
      case 0: //elevatornot down
        elevator.setGoal(0);
        break;
      case 1: //global 
        runGlobalSearchAlignment();
        break;
      case 2: // rotate
        rotateAlign();
        break;
      case 3: //local
        localAlignment();
        break;
      case 4: // raise elevator
        dt.drive(0,0,0);
        elevator.setGoal(reefLevel);
        break;
      case 5: //score
        if (reefLevel == testl4) {
          scoreL4();
        }
        else {
          regularScore();
        }
        break;
      default:
        break;
    }
  }

  /**
   * @return if within range for local alignment
   */
  private boolean detectWithinLocal() {
    /* 
     * use distance formula from center of the reef to the estimatedPosition of the robot, figure out a reasonable distance
     */
    double goodDistanceMeters = 3; //maybe 4 or even 5
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      return poseEstimator.getEstimatedPosition().getTranslation().getDistance(new Translation2d(4.500,4.000)) < goodDistanceMeters;
    }
    else if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      return poseEstimator.getEstimatedPosition().getTranslation().getDistance(new Translation2d(13.050,4.000)) < goodDistanceMeters;
    }
    else {
      DriverStation.reportError("ERM no alliance", null);
      return false; 
    }
  }

  /**
   * @return if rotation is reasonable for local alignment
   */
  private boolean checkRotation() {
    /*
     * Check estimated rotation vs wanted rotation tolerance should be 5-10 degrees 
     */
    rotationPID.enableContinuousInput(-180, 180);
    rotationPID.setSetpoint(MathUtil.inputModulus(getAngleToReefWall(), -180, 180));
    rotationPID.setTolerance(positionTolerance[2], velocityTolerance[2]);
    return rotationPID.atSetpoint();
  }

  /**
   * uses logic from MoveToAlignReef
   * @return checks if locally aligned
   */
  private boolean checkLocalAlignment() {
    /*
     * Use MoveToAlign is finished method for this
     */
    boolean aligned_with_tag = ((forwardErr <= areaTolerance) && (Math.abs(strafeErr) <= strafeTolerance));
    return (aligned_with_tag); //Aligned with tag
  }

  /**
   * Scores samo l4 style, shoot raise shoot
   */
  private void scoreL4() {
    scoringTimer.start();
    //Begin phase 1 move coral on branch
    if (scoringTimer.hasElapsed(0.7)) {
      elevator.setGoal(testl4 + testl4RaiseHeight);
      scoringTimerFinalL4.start();
      //begin phase 2 raise and score
      if (scoringTimerFinalL4.hasElapsed(0.7)) {
        coralEffector.setMotorSpeed(0);
        scoredCoral = true;
      }
      else {
        coralEffector.setMotorSpeed(0.15);
      }
    }
    else {
      coralEffector.setMotorSpeed(0.07);
    }
  }

  /**
   * Scores regular
   */
  private void regularScore() {
    scoringTimer.start();
    if (scoringTimer.hasElapsed(0.7)) {
      coralEffector.setMotorSpeed(0);
      scoredCoral = true;
    }
    else {
      coralEffector.setMotorSpeed(.07);
    }
  }

  /**
   * Uses MoveToAlignReef to calculate required speeds to align, will time  out if takes to long
   */
  private void localAlignment() {
    timeoutTimer.start();
    didntseetime.start();
    if (elevator.getCurrentHeight() <= 0.1) {
      SmartDashboard.putBoolean("LimelightFunctional", true);
      if (ll.seesTag(REEF_LL)) {
        SmartDashboard.putBoolean("SeeTag", true);
        SmartDashboard.putNumber("CurrentPercentage", LimelightHelpers.getTA(REEF_LL)); //To figure out goal
        didntseetime.reset();
        //figure out errors
        forwardErr = - LimelightHelpers.getTA(REEF_LL) + areaPercentageGoal;
        strafeErr = getStrafeErrorMeters();
        //find speeds
        double strafeSpeed = MathUtil.clamp(strafeErr*strafeSpeedMultiplier, -strafeClamp, strafeClamp);
        double forwardSpeed = MathUtil.clamp(forwardErr*forwardSpeedMultiplier, 0, forwardClamp);
        dt.drive(forwardSpeed, strafeSpeed, 0);
      }
      else {
        SmartDashboard.putBoolean("SeeTag", false);
        dt.drive(0.14, 0.14 * (rightStick ? -1 : 1), 0);
      }
    }
    else {
      SmartDashboard.putBoolean("LimelightFunctional", false);
      dt.drive(0, 0, 0);
    }
    if ((didntseetime.get() > 1.5) //Does not see tag
    || (timeoutTimer.get() > 3.0)) { //Worst case scenrio
      currentState =  State.COMPLETE;
      DriverStation.reportError("Alignment failed due to local align", false);
    }


  }
  /**
   * Supplemental method for {@link #localAlignment()}
   * @return strafeError for alignment
   */
  private double getStrafeErrorMeters(){
    return Math.sin(Units.degreesToRadians(LimelightHelpers.getTX(REEF_LL)))
    * ll.getDistanceToApriltagMT2(REEF_LL)+ (rightStick ? RIGHT_CORAL_BRANCH : LEFT_CORAL_BRANCH);
  }

  /**
   * Aligns Rotationally when 
   */
  private void rotateAlign() {
    rotationPID.enableContinuousInput(-180, 180);
    rotationPID.setSetpoint(MathUtil.inputModulus(getAngleToReefWall(), -180, 180));
    rotationPID.setTolerance(positionTolerance[2], velocityTolerance[2]);
    dt.drive(0, 0, rotationPID.calculate(poseEstimator.getEstimatedPosition().getRotation().getDegrees()));
  }
  
  private double getAngleToReefWall() {
    switch (targetID) {
      case 6:
        return 120.0;
      case 7:
        return 180.0;
      case 8:
        return -120.0;
      case 9:
        return -60.0;
      case 10:
        return 0.0;
      case 11:
        return 60.0;
      case 17:
        return 60.0;
      case 18:
        return 0.0;
      case 19:
        return -60.0;
      case 20:
        return -120.0;
      case 21:
        return 180.0;
      case 22:
        return 120.0;
      default:
        return 0.0;
    }
  }

  private int getNearestReefTag() {
    if (LimelightHelpers.getTV(REEF_LL)) {
      return (int) LimelightHelpers.getFiducialID(REEF_LL);
    }
    else {
      if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
        int sector = getSector(poseEstimator.getEstimatedPosition().getX(), 
                              poseEstimator.getEstimatedPosition().getY(), 4.5, 4.0, 150);
        return sector + 17;
      }
      else if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
        int sector = getSector(poseEstimator.getEstimatedPosition().getX(), 
                              poseEstimator.getEstimatedPosition().getY(), 13.05, 4.0, 30);
        return sector + 6;
      }
      else {
        return -1;
      }
    }
  }
  public static int getSector(double x, double y, double cx, double cy, double offsetAngle) {
    double dx = x - cx, dy = y - cy;

    if (dx == 0 && dy == 0) return -1; // at center

    double angle = -Math.atan2(dy, dx);        // clockwise angle
    if (angle < 0) angle += 2 * Math.PI;       // normalize to [0, 2π)

    double offset = Math.toRadians(offsetAngle);       // sector 0 starts at 120 degrees
    angle = (angle - offset + 2 * Math.PI) % (2 * Math.PI);

    double wedge = 2 * Math.PI / 6;            // 60 degrees per sector
    int sector = (int)(angle / wedge);         // 0–5
    if (sector == 6) sector = 5;               // rounding guard
    return sector;
  }
  /** 
   * Controls pathplanner global alignment 
   * 
   */
  private void runGlobalSearchAlignment() {
    if (currentTrajectory == null) {
      Pose2d targetLocation = calculateSearchPose();
      PathPlannerPath path = new PathPlannerPath(PathPlannerPath.waypointsFromPoses(
        List.of(poseEstimator.getEstimatedPosition(), targetLocation)),
        constraints, 
        null, 
        new GoalEndState(0, targetLocation.getRotation()));
      path.preventFlipping = true;
      // Create a trajectory manually
      currentTrajectory = path.generateTrajectory(dt.getSpeeds(), poseEstimator.getEstimatedPosition().getRotation(), Autoc.robotConfig);
      pathTimer.start();; // start time for trajectory tracking
    }
    followTrajectoryManually();
    checkCompletionOfPath();
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
  /**
   * 
   * @return Pose2d Search Pose
   */
  private Pose2d calculateSearchPose() {
    switch(targetID) {
      case 6:
        return FlippingUtil.flipFieldPose(ID6_17Search);
      case 7:
        return FlippingUtil.flipFieldPose(ID7_18Search);
      case 8:
        return FlippingUtil.flipFieldPose(ID8_19Search);
      case 9:
        return FlippingUtil.flipFieldPose(ID9_20Search);
      case 10:
        return FlippingUtil.flipFieldPose(ID10_21Search);
      case 11:
        return FlippingUtil.flipFieldPose(ID11_22Search);
      case 17:
        return ID6_17Search;
      case 18:
        return ID7_18Search;
      case 19:
        return ID8_19Search;
      case 20:
        return ID9_20Search;
      case 21:
        return ID10_21Search;
      case 22:
        return ID11_22Search;
      default:
        return new Pose2d(); 
    }
  }

}