package frc.robot.constants;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class ElectricalConstants {

    // **************************************************************
    // ************************* DRIVE ******************************
    // **************************************************************

    public static final int FRONT_RIGHT_TURN_MOTOR_ID = 1;
    public static final int FRONT_LEFT_TURN_MOTOR_ID = 4;
    public static final int BACK_LEFT_TURN_MOTOR_ID = 6;
    public static final int BACK_RIGHT_TURN_MOTOR_ID = 8;

    public static final int FRONT_RIGHT_WHEEL_MOTOR_ID = 2;
    public static final int FRONT_LEFT_WHEEL_MOTOR_ID = 3;
    public static final int BACK_LEFT_WHEEL_MOTOR_ID = 5;
    public static final int BACK_RIGHT_WHEEL_MOTOR_ID = 7;

    public static final int FRONT_LEFT_ENCODER_ID = 9;
    public static final int FRONT_RIGHT_ENCODER_ID = 10;
    public static final int BACK_LEFT_ENCODER_ID = 12;
    public static final int BACK_RIGHT_ENCODER_ID = 11;

    // **************************************************************
    // *************************** ARM ******************************
    // **************************************************************

    public static final int ARM_EXTENSION_MOTOR_ID = 17;
    public static final int ARM_ANGLE_PISTON = 2;
    public static final int ARM_GRIPPER_PISTON_FORWARD = 15;
    public static final int ARM_GRIPPER_PISTON_REVERSE = 3;
    // public static final int ARM_POSSESSION_SENSOR = ;
    public static final int ARM_ZERO_POSITION_SENSOR = 0;
    public static final int ARM_STINGER_PISTON_PORT = 14;
    public static final int ARM_STINGER_MOTOR_ID = 18;

    // **************************************************************
    // ************************ INTAKE ******************************
    // **************************************************************

    public static final int INTAKE_LEFT_MOTOR_ID = 15;
    public static final int INTAKE_RIGHT_MOTOR_ID = 16;
    public static final int INTAKE_CONFIG_PISTON_PORT = 1;
    public static final int INTAKE_PACKING_PISTON_PORT = 0;
    // public static final int POSESSION_SENSOR = 2;

    // public static final int INTAKE_BEAM_BREAK_PORT = 2;

    public static final PneumaticsModuleType PH_TYPE = PneumaticsModuleType.REVPH;
    public static final int PH_ID = 18;
    public static final int ARM_TOF_ID = 19;
    public static final int DRIVE_TOF_ID = 23;

}