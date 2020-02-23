/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import static frc.robot.Constants.PowerCellConstants.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallMagazine;
import frc.robot.subsystems.BallShooter;

public class RunShooter extends CommandBase {
  /**
   * Creates a new RunShooter.
   */
  BallShooter s_ballShooter;
  BallMagazine s_ballMagazine;
  E_SHOOT_POS pos;

  double speed;
  boolean feedBalls;


  public RunShooter(BallShooter s_ballShooter, BallMagazine s_ballMagazine, E_SHOOT_POS pos) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_ballShooter = s_ballShooter;
    this.s_ballMagazine = s_ballMagazine;
    this.pos = pos;

    feedBalls = false;

    addRequirements(s_ballShooter);
    addRequirements(s_ballMagazine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(pos == E_SHOOT_POS.CLOSE){
      speed = C_SHOOTER_SPEED_CLOSE;
    }
    else if(pos == E_SHOOT_POS.TRENCH){
      speed = C_SHOOTER_SPEED_TRENCH;
    }

    s_ballShooter.setShooterVelocityPID(speed);

    if(Math.abs(s_ballShooter.getShooterVel() - speed) < C_SHOOTER_SPEED_TOLERANCE){
      feedBalls = true;
    }

    if(feedBalls){
      s_ballMagazine.feedBall();
      s_ballShooter.setPrerollerPercent(C_PREROLLER_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_ballShooter.stopShooter();
    s_ballMagazine.stopMagazine();
    s_ballShooter.stopPreroller();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
