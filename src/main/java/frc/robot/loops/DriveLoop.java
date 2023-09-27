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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  private BigBirdRamsete ramseteControl = new BigBirdRamsete();
  private Rotation2d cmdFixedAngle = new Rotation2d();

  private Pose2d currentPose = new Pose2d();

  public enum DriveStates {
    OPERATOR_CONTROL,
    POSE_TO_POSE,
    RAMSETE,
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
  }

  public void setState(DriveStates state) {
    mDriveState = state;
  }

  @Override
  public void periodic() {
    telemInput();
    switch (mDriveState) {
      case OPERATOR_CONTROL:
        double commands[] = computeOperatorCommands(oi.getMappedDriveLeftY(), oi.getMappedDriveLeftX(),
            oi.getDriveRightX());
        mDriveRequest = new SwerveRequest.FieldCentric()
            .withIsOpenLoop(true)
            .withVelocityX(commands[0])
            .withVelocityY(commands[1])
            .withRotationalRate(commands[2]);
        break;

      case RAMSETE:
        ChassisSpeeds goalSpeeds = ramseteControl.getGoalSpeeds(currentPose); // Chassis speeds as dictated
                                                                              // by Ramsete
        /*
         * Keep in mind that the ramsete controller is for a uni-cycle model. In a
         * swerve's case we cannot apply the vx, vy & omega directly, as our heading
         * and doesnt help us track a path. This path follower doesnt actually control
         * our
         * swerve angle
         * 
         * Instead treat the chassis speeds like a polar vector,
         * (unicycles have vy = 0 always so thats what enables this)
         * vx -> radius (magnitude)
         * omega -> angle
         * 
         * Once we have a polar vector we can convert to cartesian, and get a vx & vy
         * that the swerve will move to
         */
        Translation2d velVector = new Translation2d(goalSpeeds.vxMetersPerSecond,
            Rotation2d.fromDegrees(goalSpeeds.omegaRadiansPerSecond)); // Creating our polar vector

        // The constructor automatically does the conversion to store cartesian
        // components so we just need to use the getters
        double vx = velVector.getX();
        double vy = velVector.getY();

        // Now we can send these new vx & vy commands
        mDriveRequest = new SwerveRequest.FieldCentricFacingAngle()
            .withIsOpenLoop(false)
            .withVelocityX(vx)
            .withVelocityY(vy)
            .withTargetDirection(cmdFixedAngle);
        SmartDashboard.putNumber("cmdVX", vx);
        SmartDashboard.putNumber("cmdVY", vy);
        SmartDashboard.putNumber("angleReq", cmdFixedAngle.getDegrees());
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
    mDrive.updateSimState(0.02, 12);
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

    SmartDashboard.putNumber("cmdX", cmdX);
    SmartDashboard.putNumber("cmdY", cmdY);
    SmartDashboard.putNumber("cmdOmega", cmdOmega);

    double[] ret = { cmdX, cmdY, cmdOmega };

    SmartDashboard.putNumberArray("CMD Speeds", ret);

    return ret;
    // }
  }

  private void telemeterize(SwerveDriveState state) {
    field2d.setRobotPose(state.Pose);
    // field2d.getObject("Waypoints").setPoses(new Pose2d(0., 0., new Rotation2d()),
    // new Pose2d(5, 3, new Rotation2d()));
    field2d.getObject("Traj").setTrajectory(ramseteControl.getCurrentTrajectory());
    SmartDashboard.putString("DriveState", mDriveState.toString());
    currentPose = state.Pose;
  }

  private void telemInput() {
    double teleB = Preferences.getDouble("Ram_B", 2.0);
    double teleZ = Preferences.getDouble("Ram_Zeta", 0.7);
    if (teleB != ramseteControl.getB() || teleZ != ramseteControl.getZeta()) {
      ramseteControl.setBZeta(teleB, teleZ);
      System.out.println("Updated Ramsete");
    }
  }

  public void updateTrajectory(String filename) {
    Trajectory newTrajectory = ramseteControl.readTrajectory(filename);
    ramseteControl.setTrajectory(newTrajectory);
    ramseteControl.resetPath();
  }

  public Trajectory getTrajectory() {
    return ramseteControl.getCurrentTrajectory();
  }

  public void setFixedAngle(Rotation2d rot) {
    cmdFixedAngle = rot;
  }

  public boolean atPathEnd() {
    return ramseteControl.atPathEnd(currentPose);
  }

  public void resetOdomFieldRelative(Pose2d newPose) {
    mDrive.seedFieldRelative(newPose);
  }

  public void updateRamseteGains(double b, double zeta) {
    ramseteControl.setBZeta(b, zeta);
  }

}
