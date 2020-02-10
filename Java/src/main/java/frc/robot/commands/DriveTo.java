/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.C_kD_drive;
import static frc.robot.Constants.DriveConstants.C_kI_drive;
import static frc.robot.Constants.DriveConstants.C_kP_drive;
import static frc.robot.Constants.DriveConstants.C_maxAccel_drive;
import static frc.robot.Constants.DriveConstants.C_maxVel_drive;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.DriveTrain;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class DriveTo extends ProfiledPIDCommand {
  /**
   * Creates a new DriveTo.
   */
  public DriveTo(double targetDistance, DriveTrain s_dt) {
    super(
        new ProfiledPIDController(

            C_kP_drive, C_kI_drive, C_kD_drive,                                        // The PID gains
            new TrapezoidProfile.Constraints(C_maxVel_drive, C_maxAccel_drive)),       // The motion profile constraints
        
        () -> s_dt.averageEncoderGet(),                             // This should return the measurement
        () -> targetDistance,                                       // This should return the goal (can also be a constant)
        (output, setpoint) -> { s_dt.driveWheels(output, output);   // This uses the output
        });    
        
     // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_dt);
  
    // Configure additional PID options by calling `getController` here.
    //getController().enableContinuousInput(-180, 180);
    //getController().setTolerance(2.5);
  
  }

  // Returns true if at setpoint
  public boolean atSetpoint(){
    return getController().atSetpoint();
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
