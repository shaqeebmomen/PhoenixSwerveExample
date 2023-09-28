package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.control.P2PTrajectory;
import frc.robot.control.P2PWaypoint;

public class LocationConstants {

    public enum AllianceColour {
        RED,
        BLUE
    }

    public static P2PTrajectory testTrajectory = new P2PTrajectory(new P2PWaypoint[] {
            // new P2PWaypoint(new Pose2d(0, 2, new Rotation2d()), 1),
            new P2PWaypoint(new Pose2d(2, 1, new Rotation2d()), 0.1),
            new P2PWaypoint(new Pose2d(5, 2, new Rotation2d()), 0.1),
            new P2PWaypoint(new Pose2d(2, 5, new Rotation2d()), 0)
    });

}
