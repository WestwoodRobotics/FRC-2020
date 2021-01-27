/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveDistanceBad extends CommandBase {
  /**
   * Creates a new DriveDistanceTime.
   */

  DriveTrain s_driveTrain;
  double meters;
  double percent;

  public DriveDistanceBad(double meters, double percent, DriveTrain s_driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_driveTrain = s_driveTrain;
    this.meters = meters;
    this.percent = percent;
    addRequirements(s_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_driveTrain.zeroLeftEncoder();
    s_driveTrain.zeroRightEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_driveTrain.driveWheelsPercent(percent, percent);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return s_driveTrain.getAverageEncoderMeters() >= meters;
  }
}
