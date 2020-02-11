/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.DriveTrain;



// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ProfiledTurnTo extends ProfiledPIDCommand {
  /**
   * Creates a new ProfiledTurnTo.
   */

  public ProfiledTurnTo(double degrees, DriveTrain s_dt) {
    super(
      // The ProfiledPIDController used by the command
      new ProfiledPIDController(
          // The PID gains
          C_kP_turn, C_kI_turn, C_kD_turn,
          
          // The motion profile constraints
          new TrapezoidProfile.Constraints(C_maxVel_turn, C_maxAccel_turn)),
      
          // This should return the measurement
      s_dt::getHeading,
      
      // This should return the goal (can also be a constant)
      () -> new TrapezoidProfile.State(degrees, 0),
      
      // This uses the output
      (output, setpoint) -> {
        s_dt.driveWheelsPercent(output, output);
      },
      s_dt
    );
    
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(s_dt);

    this.getController().enableContinuousInput(-180, 180);
    this.getController().setTolerance(2.5, 2);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.getController().atGoal();
  }
}
