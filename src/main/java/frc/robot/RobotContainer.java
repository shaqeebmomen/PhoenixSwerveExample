// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.constants.LocationConstants.AllianceColour;
import frc.robot.loops.DriveLoop;
import frc.robot.loops.DriveLoop.DriveStates;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private DriveLoop driveLoop;

    private static SendableChooser<AllianceColour> allianceChooser = new SendableChooser<>();

    public RobotContainer() {
        driveLoop = DriveLoop.getInstance();
    }

    public static AllianceColour getAllianceColour() {
        return allianceChooser.getSelected();
    }

    public void initAuto() {
        driveLoop.setState(DriveStates.DISABLED);

    }

    public void initTeleop() {
        driveLoop.setState(DriveStates.OPERATOR_CONTROL);

    }

    public void initDisabled() {
        driveLoop.setState(DriveStates.DISABLED);
    }
}
