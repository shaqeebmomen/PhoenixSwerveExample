package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.loops.DriveLoop;
import frc.robot.util.OI;

public class RobotControl {
    private OI oi;
    private DriveLoop drive;

    public RobotControl() {
        oi = OI.getInstance();
        drive = DriveLoop.getInstance();
    }

    

}
