package frc.robot.control;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

public class P2PPathController {
    private PIDController pxController; // posiiton x controller
    private PIDController pyController; // position y controller
    private PIDController thetaController; // heading controller;
    private SlewRateLimiter vxSlewLimiter;
    private SlewRateLimiter vySlewLimiter;
    private SlewRateLimiter omegaSlewLimiter;
    public P2PTrajectory currentTrajectory;
    public Pose2d currentPose;

    // Constructor
    public P2PPathController(P2PTrajectory trajectory, double positionkP, double positionkI, double positionkD,
            double positionTolerance,
            double thetakP, double thetakI, double thetakD, double thetaTolerance, double slewRateTranslation,
            double slewRateRotation) {

        pxController = new PIDController(positionkP, positionkI, positionkD);
        pxController.setTolerance(positionTolerance);

        pyController = new PIDController(positionkP, positionkI, positionkD);
        pyController.setTolerance(positionTolerance);

        thetaController = new PIDController(thetakP, thetakI, thetakD);
        thetaController.setTolerance(thetaTolerance);

        vxSlewLimiter = new SlewRateLimiter(slewRateTranslation);
        vySlewLimiter = new SlewRateLimiter(slewRateTranslation);
        omegaSlewLimiter = new SlewRateLimiter(slewRateRotation);

        currentTrajectory = trajectory;

    }

    // *************** Calculation Methods ***************

    // TODO either make a quick drive loop example in this project or document how a
    // user is expected to use this class, its not obvious that all they need to do
    // is constuct and call this method repeatedly
    /**
     * 
     * @param currentPose      current position of robot (Pose2d)
     * @param velocitySetpoint commanded velocity of robot (m/s)
     * @return chassis speeds to get to the waypoint
     */
    public ChassisSpeeds getGoalSpeeds(Pose2d currentPose, double velocitySetpoint) {
        setCurrentPose(currentPose);
        if (currentTrajectory.isCurrentEndPoint()) {
            return PoseToPoseControl();
        } else {
            return VelocityHeadingControl(velocitySetpoint);
        }
    }

    /**
     * 
     * @param currentPose
     * @return chassis speeds using normal pose to pose method
     */
    private ChassisSpeeds PoseToPoseControl() {
        Pose2d targetPose = currentTrajectory.getCurrentWaypoint().getPose();

        double vx = vxSlewLimiter.calculate(pxController.calculate(currentPose.getX(), targetPose.getX()));
        double vy = vySlewLimiter.calculate(pyController.calculate(currentPose.getY(), targetPose.getY()));
        double omega = omegaSlewLimiter.calculate(thetaController.calculate(currentPose.getRotation().getRadians(),
                targetPose.getRotation().getRadians()));

        return new ChassisSpeeds(vx, vy, omega);
    }

    /**
     * 
     * @param currentPose
     * @param velocitySetpoint
     * @return chassis speeds using constant velocity
     */
    private ChassisSpeeds VelocityHeadingControl(double velocitySetpoint) {
        // TODO maybe move this to the getGoalSpeeds calculations, this function might be doing too much
        if (inSetpointRadius()) {
            currentTrajectory.nextWaypoint();
        }

        Pose2d targetPose = currentTrajectory.getCurrentWaypoint().getPose();
        double targetAngle = getAngleToWaypoint();

        double vy = vySlewLimiter.calculate(velocitySetpoint * Math.sin(targetAngle));
        double vx = vxSlewLimiter.calculate(velocitySetpoint * Math.cos(targetAngle));
        double omega = omegaSlewLimiter.calculate(thetaController
                .calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians()));

        return new ChassisSpeeds(vx, vy, omega);
    }

    // *************** Util Methods ***************
    /**
     * 
     * @param currentPose
     * @return distance of robot to the next waypoint
     */
    private double getDistanceToWaypoint() {
        Pose2d waypoint = currentTrajectory.getCurrentWaypoint().getPose();
        double dx = waypoint.getX() - currentPose.getX();
        double dy = waypoint.getY() - currentPose.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    /**
     * 
     * @return true when the robot meets the end condition
     */
    private boolean inSetpointRadius() {
        return getDistanceToWaypoint() < currentTrajectory.getCurrentWaypoint().getEndRadius();
    }

    /**
     * 
     * @return angular velocity for robot to get to the next waypoint
     */
    private double getAngleToWaypoint() {
        Pose2d targetPose = currentTrajectory.getCurrentWaypoint().getPose();

        double y = targetPose.getY() - currentPose.getY();
        double x = targetPose.getX() - currentPose.getY();

        double angleToWaypoint = Units.radiansToDegrees(Math.atan2(y, x));

        return angleToWaypoint;
    }

    // *************** Other Methods ***************

    /**
     * 
     * @param currentPose
     * @return true when the pid is settled
     */
    public boolean isSettled() {
        return pxController.atSetpoint() && pyController.atSetpoint() && thetaController.atSetpoint()
                && currentTrajectory.isCurrentEndPoint();
    }

    /**
     * 
     * @param newTrajectory set current trajectory to the new one
     */
    public void setTrajectory(P2PTrajectory newTrajectory) {
        this.currentTrajectory = newTrajectory;
    }

    // *************** Configs ***************

    public void setPositionPID(double kP, double kI, double kD) {
        pxController.setPID(kP, kI, kD);
        pyController.setPID(kP, kI, kD);
    }

    public void setThetaPID(double kP, double kI, double kD) {
        thetaController.setPID(kP, kI, kD);
    }

    public void setVelocitySlewRate(double rate) {
        vxSlewLimiter = new SlewRateLimiter(rate);
        vySlewLimiter = new SlewRateLimiter(rate);
    }

    public void setOmegaSlewRate(double rate) {
        vxSlewLimiter = new SlewRateLimiter(rate);
        vySlewLimiter = new SlewRateLimiter(rate);
    }

    // ********* Setters and Resetters ************
    private void setCurrentPose(Pose2d currentPose) {
        this.currentPose = currentPose;
    }

    public void resetVxSlewRateLimiter() {
        vxSlewLimiter.reset(0);
    }

    public void resetVySlewRateLimiter() {
        vySlewLimiter.reset(0);
    }

    public void resetOmegaSlewRateLimiter() {
        omegaSlewLimiter.reset(0);
    }

    public void resetPxController() {
        pxController.reset();
    }

    public void resetPyController() {
        pyController.reset();
    }

    public void resetThetaController() {
        thetaController.reset();
    }

    public void reset() {
        resetOmegaSlewRateLimiter();
        resetPxController();
        resetPyController();
        resetThetaController();
        resetVxSlewRateLimiter();
        resetVySlewRateLimiter();

        currentTrajectory.reset();
    }

}