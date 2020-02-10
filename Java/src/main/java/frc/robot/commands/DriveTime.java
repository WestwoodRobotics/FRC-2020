/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveTime extends CommandBase {
  /**
   * Creates a new DriveTime.
   */

  private DriveTrain s_dt;
  private double time;
  private Timer timer;

  public DriveTime(double t, DriveTrain s_dt) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_dt = s_dt;
    time = t;
    addRequirements(s_dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_dt.driveWheels(0.5, 0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_dt.stopWheels();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() == time;
  }
}
