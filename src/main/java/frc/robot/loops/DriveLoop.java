// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.loops;

import java.util.List;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.LocationConstants.AllianceColour;
import frc.robot.subsystems.Drive;
import frc.robot.util.OI;

/** Add your docs here. */
public class DriveLoop extends SubsystemBase {

  private static DriveLoop mInstance = null;
  private DriveStates mDriveState;
  private OI oi;
  private Drive mDrive;
  final Field2d field2d = new Field2d();

  public enum DriveStates {
    OPERATOR_CONTROL,
    POSE_TO_POSE,
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
    field2d.getObject("Waypoints").setPoses(new Pose2d(0., 0.,new Rotation2d()), new Pose2d(5, 3, new Rotation2d()));
  }

}
