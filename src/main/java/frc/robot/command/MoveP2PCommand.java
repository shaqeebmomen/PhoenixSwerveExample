// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.LocationConstants;
import frc.robot.control.P2PTrajectory;
import frc.robot.control.P2PWaypoint;
import frc.robot.loops.DriveLoop;
import frc.robot.loops.DriveLoop.DriveStates;

public class MoveP2PCommand extends CommandBase {

  private DriveLoop drive;
  private P2PTrajectory trajectory;

  /** Creates a new MovePathRamsete. */
  public MoveP2PCommand() {
    drive = DriveLoop.getInstance();
    trajectory = LocationConstants.testTrajectory;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.resetPathController();
    drive.setCurrentTrajectory(trajectory);
    drive.resetOdomFieldRelative(new Pose2d());
    drive.setPathCruiseSpeed(2);
    drive.setState(DriveStates.PATH_FOLLOW);
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
