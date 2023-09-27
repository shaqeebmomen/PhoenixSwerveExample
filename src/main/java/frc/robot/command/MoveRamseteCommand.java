// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.loops.DriveLoop;
import frc.robot.loops.DriveLoop.DriveStates;

public class MoveRamseteCommand extends CommandBase {

  private DriveLoop drive;

  /** Creates a new MovePathRamsete. */
  public MoveRamseteCommand() {
    drive = DriveLoop.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    drive.updateTrajectory("paths/output/test.wpilib.json");
    drive.setFixedAngle(new Rotation2d()); // Whatever end angle you want
    drive.resetOdomFieldRelative(drive.getTrajectory().getInitialPose());
    drive.setState(DriveStates.RAMSETE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.setState(DriveStates.OPERATOR_CONTROL);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return false;
    return drive.atPathEnd();
  }
}
