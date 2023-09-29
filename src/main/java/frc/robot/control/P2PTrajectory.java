package frc.robot.control;

import java.lang.reflect.Array;

import edu.wpi.first.math.geometry.Pose2d;

public class P2PTrajectory {

    private P2PWaypoint[] waypoints;
    private int waypointIndex;

    // constructor
    public P2PTrajectory(P2PWaypoint[] wayPoints) {
        this.waypoints = wayPoints;
    }

    /**
     * 
     * @return current waypoint
     */
    public P2PWaypoint getCurrentWaypoint() {
        return waypoints[waypointIndex];
    }

    /**
     * change to next waypoint
     */
    public void nextWaypoint() {
        if (waypointIndex + 1 < waypoints.length) {
            waypointIndex++;
        }
    }

    // ************** Getters & Setters **************
    public P2PWaypoint[] getWaypoints() {
        return waypoints;
    }

    public Pose2d[] getPoses() {
        Pose2d[] poses = new Pose2d[waypoints.length];
        int index = 0;
        for (P2PWaypoint waypoint : waypoints) {
            poses[index] = waypoint.getPose();
            index++;
        }
        return poses;
    }

    public P2PWaypoint getWayPoint(int index) {
        if (index >= 0 && index < waypoints.length) {
            return waypoints[index];
        }
        return null;
    }

    public void setWaypoints(P2PWaypoint[] waypoints) {
        this.waypoints = waypoints;
    }

    public void setWaypoint(P2PWaypoint waypoint, int index) {
        if (index >= 0 && index < waypoints.length) {
            waypoints[index] = waypoint;
        }
    }

    public void reset() {
        this.waypointIndex = 0;
    }

    /**
     * 
     * @return true when the waypoint is the last one in the trajectory
     */
    public boolean isCurrentEndPoint() {
        return waypointIndex == waypoints.length - 1;
    }

}
