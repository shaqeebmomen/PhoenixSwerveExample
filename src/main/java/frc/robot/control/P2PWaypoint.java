package frc.robot.control;

import edu.wpi.first.math.geometry.Pose2d;

// waypoint class
public class P2PWaypoint {
    private Pose2d pose;
    private double endRadius;

    public P2PWaypoint(Pose2d pose, double endRadius) {
        this.pose = pose;
        this.endRadius = endRadius;
    }

    public void setEndRadius(double endRadius) {
        this.endRadius = endRadius;
    }

    public double getEndRadius() {
        return endRadius;
    }

    public Pose2d getPose() {
        return pose;
    }

    public void setPose(Pose2d pose) {
        this.pose = pose;
    }

}