package frc.robot.control;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;

public class BigBirdRamsete {
    RamseteController ramseteController;
    Trajectory currentTrajectory;

    private double timeSinceStart;
    private double previousTime;
    private double currentTime;
    private double tolerancePos = 0.05; // 5 cm
    private double toleranceRot = Units.degreesToRadians(1); // 1 deg

    /**
     * initializes the trajectory, the b gain, and the zeta gain
     * 
     * @param trajectory
     * @param b
     * @param zeta
     */
    public BigBirdRamsete(Trajectory trajectory, double b, double zeta) {
        ramseteController = new RamseteController(b, zeta);
        setTrajectory(trajectory);
        resetPath();
        timeSinceStart = 0;
    }

    public BigBirdRamsete() {
        setTrajectory(new Trajectory());
        ramseteController = new RamseteController();
        timeSinceStart = 0;
    }

    /**
     * resets the time variables
     * must be called before the getGoalSpeeds method
     */
    public void resetPath() {
        timeSinceStart = 0;
        previousTime = Timer.getFPGATimestamp();
        currentTime = Timer.getFPGATimestamp();
    }

    /**
     * resetPath method should be called before getGoalSpeeds in order for
     * timeSinceStart to be accumulated
     * calculates velocity needed to reach setpoint
     * 
     * @param currentPose
     * @return adjusted speeds
     */
    public ChassisSpeeds getGoalSpeeds(Pose2d currentPose) {
        currentTime = Timer.getFPGATimestamp();
        double deltaTime = currentTime - previousTime;
        previousTime = currentTime;
        timeSinceStart += deltaTime;

        Trajectory.State goal = currentTrajectory.sample(timeSinceStart);
        ChassisSpeeds adjustedSpeeds = ramseteController.calculate(currentPose, goal);
        return adjustedSpeeds;
    }

    /**
     * Gets JSON File from Pathweaver and assigns it to trajectory
     * 
     * @param filename Name of JSON File acquired from Pathweaver
     * @return currentTrajectory
     */
    public Trajectory readTrajectory(String filename) {
        Trajectory trajectory = null;
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException exception) {
            DriverStation.reportError(
                    "Unable to open trajectory " + filename + "\nwith message: " + exception.getMessage(),
                    exception.getStackTrace());
        }
        return trajectory;
    }

    /**
     * returns currentTrajectory
     * 
     * @return currentTrajectory
     */
    public Trajectory getCurrentTrajectory() {
        return currentTrajectory;
    }

    /**
     * sets currentTrajectory to newTrajectory
     * 
     * @param newTrajectory
     */
    public void setTrajectory(Trajectory newTrajectory) {
        currentTrajectory = newTrajectory;
    }

    /**
     * sets b and zeta gains
     * 
     * @param b
     * @param zeta
     */
    public void setBZeta(double b, double zeta) {
        ramseteController = new RamseteController(b, zeta);
    }

    // TODO this doesn't work if our path crosses over our final point, we need some
    // index of "progress" along our path as well, that ideally isnt time
    public boolean atPathEnd(Pose2d currentPose2d) {
        List<Trajectory.State> trajectoryStates = currentTrajectory.getStates();
        Trajectory.State lastState = trajectoryStates.get(trajectoryStates.size() - 1);
        Pose2d finalPose = lastState.poseMeters;
        boolean atX = Math.abs(finalPose.getX() - currentPose2d.getX()) < tolerancePos;
        boolean atY = Math.abs(finalPose.getY() - currentPose2d.getY()) < tolerancePos;
        boolean atTheta = (finalPose.getRotation().minus(currentPose2d.getRotation())).getDegrees() < toleranceRot;
        boolean atEnd = timeSinceStart > 0.95 * currentTrajectory.getTotalTimeSeconds();
        return atX && atY && atTheta && atEnd;
    }
}
