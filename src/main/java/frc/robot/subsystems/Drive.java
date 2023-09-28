// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.constants.DriveConstants;
import frc.robot.constants.DriveConstants.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SwerveModuleSteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;

public class Drive extends SwerveDrivetrain {

  static Drive mInstance = null;

  static class CustomSlotGains extends Slot0Configs {
    public CustomSlotGains(double kP, double kI, double kD, double kV, double kS) {
      this.kP = kP;
      this.kI = kI;
      this.kD = kD;
      this.kV = kV;
      this.kS = kS;
    }
  }

  private static final CustomSlotGains steerGains = new CustomSlotGains(50, 0, 0.05, 0, 0);
  private static final CustomSlotGains driveGains = new CustomSlotGains(3, 0, 0, 0, 0);

  private static final double kCoupleRatio = 0.0;

  private static final double kDriveGearRatio = 6.056;
  private static final double kSteerGearRatio = 12.8;
  private static final double kWheelRadiusInches = 2;
  private static final int kPigeonId = 1;
  private static final boolean kSteerMotorReversed = false;
  private static final String kCANbusName = "";
  private static final boolean kInvertLeftSide = false;
  private static final boolean kInvertRightSide = false;

  private static double kSteerInertia = 0.0001;
  private static double kDriveInertia = 0.001;

  private static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
      .withPigeon2Id(kPigeonId)
      .withSupportsPro(false)
      .withCANbusName(kCANbusName);

  private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
      .withDriveMotorGearRatio(kDriveGearRatio)
      .withSteerMotorGearRatio(kSteerGearRatio)
      .withWheelRadius(kWheelRadiusInches)
      .withSlipCurrent(800)
      .withSteerMotorGains(steerGains)
      .withDriveMotorGains(driveGains)
      .withSpeedAt12VoltsMps(6) // Theoretical free speed is 10 meters per second at 12v applied output
      .withSteerInertia(kSteerInertia)
      .withDriveInertia(kDriveInertia)
      // .withFeedbackSource(SwerveModuleSteerFeedbackType.FusedCANcoder)
      .withFeedbackSource(SwerveModuleSteerFeedbackType.RemoteCANcoder)
      .withCouplingGearRatio(kCoupleRatio) // Every 1 rotation of the azimuth results in couple ratio drive turns
      .withSteerMotorInverted(kSteerMotorReversed);

  private static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
      DriveConstants.kFrontLeftSteerMotorId, DriveConstants.kFrontLeftDriveMotorId, DriveConstants.kFrontLeftEncoderId,
      DriveConstants.kFrontLeftEncoderOffset,
      Units.inchesToMeters(DriveConstants.kFrontLeftXPosInches),
      Units.inchesToMeters(DriveConstants.kFrontLeftYPosInches), kInvertLeftSide);

  private static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
      DriveConstants.kFrontRightSteerMotorId, DriveConstants.kFrontRightDriveMotorId,
      DriveConstants.kFrontRightEncoderId, DriveConstants.kFrontRightEncoderOffset,
      Units.inchesToMeters(DriveConstants.kFrontRightXPosInches),
      Units.inchesToMeters(DriveConstants.kFrontRightYPosInches), kInvertRightSide);

  private static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
      DriveConstants.kBackLeftSteerMotorId, DriveConstants.kBackLeftDriveMotorId, DriveConstants.kBackLeftEncoderId,
      DriveConstants.kBackLeftEncoderOffset,
      Units.inchesToMeters(DriveConstants.kBackLeftXPosInches),
      Units.inchesToMeters(DriveConstants.kBackLeftYPosInches), kInvertLeftSide);

  private static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
      DriveConstants.kBackRightSteerMotorId, DriveConstants.kBackRightDriveMotorId, DriveConstants.kBackRightEncoderId,
      DriveConstants.kBackRightEncoderOffset,
      Units.inchesToMeters(DriveConstants.kBackRightXPosInches),
      Units.inchesToMeters(DriveConstants.kBackRightYPosInches), kInvertRightSide);

  public Drive() {
    super(DrivetrainConstants, 100, FrontLeft,
        FrontRight, BackLeft, BackRight);
  }

}
