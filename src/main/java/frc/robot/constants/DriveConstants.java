package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class DriveConstants {
    public static final double DRIVE_WHEELBASE = Units.inchesToMeters(29 - (2 * 2.625));
    public static final double DRIVE_TRACKWIDTH = Units.inchesToMeters(26 - (2 * 2.625));

    // Copied from example
    public static final int kFrontLeftDriveMotorId = 1;
    public static final int kFrontLeftSteerMotorId = 0;
    public static final int kFrontLeftEncoderId = 0;
    public static final double kFrontLeftEncoderOffset = -0.75;
    public static final double kFrontLeftXPosInches = Units.metersToInches(DRIVE_TRACKWIDTH / 2);
    public static final double kFrontLeftYPosInches = Units.metersToInches(DRIVE_WHEELBASE / 2);

    public static final int kFrontRightDriveMotorId = 3;
    public static final int kFrontRightSteerMotorId = 2;
    public static final int kFrontRightEncoderId = 1;
    public static final double kFrontRightEncoderOffset = -0.75;
    public static final double kFrontRightXPosInches = Units.metersToInches(DRIVE_TRACKWIDTH / 2);
    public static final double kFrontRightYPosInches = -Units.metersToInches(DRIVE_WHEELBASE / 2);

    public static final int kBackLeftDriveMotorId = 5;
    public static final int kBackLeftSteerMotorId = 4;
    public static final int kBackLeftEncoderId = 2;
    public static final double kBackLeftEncoderOffset = -0.75;
    public static final double kBackLeftXPosInches = -Units.metersToInches(DRIVE_TRACKWIDTH / 2);
    public static final double kBackLeftYPosInches = Units.metersToInches(DRIVE_WHEELBASE / 2);

    public static final int kBackRightDriveMotorId = 7;
    public static final int kBackRightSteerMotorId = 6;
    public static final int kBackRightEncoderId = 3;
    public static final double kBackRightEncoderOffset = -0.75;
    public static final double kBackRightXPosInches = -Units.metersToInches(DRIVE_TRACKWIDTH / 2);
    public static final double kBackRightYPosInches = -Units.metersToInches(DRIVE_WHEELBASE / 2);

    // Drive motion constraints
    public static final double MODULE_MAX_WHEEL_SPEED_M = 4.96; // m/s, from SDS website
    public static final double DRIVE_MAX_VX = MODULE_MAX_WHEEL_SPEED_M;
    public static final double DRIVE_MAX_VY = MODULE_MAX_WHEEL_SPEED_M;
    public static final double DRIVE_MAX_OMEGA = MODULE_MAX_WHEEL_SPEED_M /
            (Math.sqrt(Math.pow(DRIVE_TRACKWIDTH / 2, 2) + Math.pow(DRIVE_WHEELBASE / 2, 2))); // 12.4
                                                                                               // rads/s
}