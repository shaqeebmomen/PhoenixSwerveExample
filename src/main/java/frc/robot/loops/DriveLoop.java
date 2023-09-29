// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.loops;

import java.util.List;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.control.P2PPathController;
import frc.robot.control.P2PTrajectory;
import frc.robot.control.P2PWaypoint;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.LocationConstants.AllianceColour;
import frc.robot.control.BigBirdRamsete;
import frc.robot.subsystems.Drive;
import frc.robot.util.OI;

/** Add your docs here. */
public class DriveLoop extends SubsystemBase {

  private static DriveLoop mInstance = null;
  private DriveStates mDriveState;
  private OI oi;
  private Drive mDrive;
  final Field2d field2d = new Field2d();

  private ChassisSpeeds commandedSpeeds;

  private double simLastTS;

  // Pathing
  private P2PPathController pathController;
  private double pathCruiseSpeed;

  private Pose2d currentPose = new Pose2d();

  public enum DriveStates {
    OPERATOR_CONTROL,
    POSE_TO_POSE,
    PATH_FOLLOW,
    DISABLED
  }

  private SwerveRequest mDriveRequest = new SwerveRequest.Idle();

  public static DriveLoop getInstance() {
    if (mInstance == null) {
      mInstance = new DriveLoop();
    }
    return mInstance;
  }

  private DriveLoop() {
    SmartDashboard.putData(field2d);
    oi = OI.getInstance();
    mDriveState = DriveStates.DISABLED;
    mDrive = new Drive();
    mDrive.registerTelemetry(this::telemeterize);

    pathController = new P2PPathController(new P2PTrajectory(new P2PWaypoint[] { new P2PWaypoint(new Pose2d(), 0) }),
        1,
        0, 0, 0.05,
        1, 0, 0, 1, 5, 5);

    commandedSpeeds = new ChassisSpeeds(0, 0, 0);
    if (Robot.isSimulation()) {
      simLastTS = Timer.getFPGATimestamp();
    }
  }

  public void setState(DriveStates state) {
    mDriveState = state;
  }

  @Override
  public void periodic() {
    telemInput();
    telemeterizeSlow();
    switch (mDriveState) {
      case OPERATOR_CONTROL:
        double commands[] = computeOperatorCommands(
            oi.getDriveLeftY(),
            oi.getDriveLeftX(),
            oi.getDriveRightX());
        commandedSpeeds = new ChassisSpeeds(commands[0], commands[1], commands[2]);
        mDriveRequest = new SwerveRequest.FieldCentric()
            .withIsOpenLoop(false)
            .withVelocityX(commandedSpeeds.vxMetersPerSecond)
            .withVelocityY(commandedSpeeds.vyMetersPerSecond)
            .withRotationalRate(commandedSpeeds.omegaRadiansPerSecond);

        break;

      case PATH_FOLLOW:
        commandedSpeeds = pathController.getGoalSpeeds(currentPose, pathCruiseSpeed);
        // Now we can send these new vx & vy commands
        mDriveRequest = new SwerveRequest.FieldCentric()
            .withIsOpenLoop(false)
            .withVelocityX(commandedSpeeds.vxMetersPerSecond)
            .withVelocityY(commandedSpeeds.vyMetersPerSecond)
            .withRotationalRate(commandedSpeeds.omegaRadiansPerSecond);
        break;

      default:
      case DISABLED:
        mDriveRequest = new SwerveRequest.Idle();
        break;

    }
    mDrive.setControl(mDriveRequest);
  }

  @Override
  public void simulationPeriodic() {
    mDrive.updateSimState(Timer.getFPGATimestamp() - simLastTS, 12);
    simLastTS = Timer.getFPGATimestamp();
  }

  private double[] computeOperatorCommands(double vx, double vy, double omega) {
    // if (vx == 0 & vy == 0 & omega == 0) {
    // return null;
    // } else {
    double cmdX;
    double cmdY;
    double cmdOmega;

    cmdX = vx * DriveConstants.DRIVE_MAX_VX
        * Preferences.getDouble("OPEN_LOOP_GAIN", 1);
    cmdY = vy * DriveConstants.DRIVE_MAX_VY
        * Preferences.getDouble("OPEN_LOOP_GAIN", 1);
    cmdOmega = omega * DriveConstants.DRIVE_MAX_OMEGA
        * Preferences.getDouble("OPEN_LOOP_GAIN", 1);

    double[] ret = { cmdX, cmdY, cmdOmega };
    return ret;
    // }
  }

  private void telemeterize(SwerveDriveState state) {
    currentPose = state.Pose;
  }

  private void telemeterizeSlow() {
    field2d.setRobotPose(currentPose);
    field2d.getObject("Traj").setPoses(pathController.currentTrajectory.getPoses());
    SmartDashboard.putString("DriveState", mDriveState.toString());
    SmartDashboard.putNumber("PoseX", currentPose.getX());
    SmartDashboard.putNumber("PoseY", currentPose.getY());

    Pose2d targetPose = pathController.currentTrajectory.getCurrentWaypoint().getPose();
    field2d.getObject("Target").setPose(targetPose);
    SmartDashboard.putNumber("TgtX", targetPose.getX());
    SmartDashboard.putNumber("TgtY", targetPose.getY());

    SmartDashboard.putNumber("cmdVX", commandedSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("cmdVY", commandedSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("cmdOM", commandedSpeeds.omegaRadiansPerSecond);
  }

  private void telemInput() {
  }

  public void resetPathController() {
    pathController.reset();
  }

  /******* GETTTERS *******/

  public P2PTrajectory getCurrentTrajectory() {
    return pathController.currentTrajectory;
  }

  public double getPathCruiseSpeed() {
    return pathCruiseSpeed;
  }

  public boolean atPathEnd() {
    return pathController.isSettled();
  }

  /****** SETTERS *******/

  public void setCurrentTrajectory(P2PTrajectory currentTrajectory) {
    pathController.setTrajectory(currentTrajectory);
    resetPathController();
  }

  public void resetOdomFieldRelative(Pose2d newPose) {
    mDrive.seedFieldRelative(newPose);
  }

  public void setPathCruiseSpeed(double pathCruiseSpeed) {
    this.pathCruiseSpeed = pathCruiseSpeed;
  }

}
