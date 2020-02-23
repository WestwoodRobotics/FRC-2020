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
  
  //--------------------------------------------------------------------------------------------------
  // Constructor

  public ProfiledTurnTo(double degrees, DriveTrain s_driveTrain) {
    super(
      // The ProfiledPIDController used by the command
      new ProfiledPIDController(
        C_kP_turn, C_kI_turn, C_kD_turn,                                          // The PID gains
        new TrapezoidProfile.Constraints(                                         // The motion profile constraints
                        radiansToMeters(C_maxVel_turn),       // Max Velocity
                        radiansToMeters(C_maxAccel_turn)      // Max Acceleration
                      )
      ),
      s_driveTrain::getHeadingRadians,                                            // This should return the measurement
      () -> new TrapezoidProfile.State(Math.toRadians(degrees), 0),               // This should return the goal (can also be a constant)
      
      (output, setpoint) -> {                                                     // This uses the output

        ChassisSpeeds chassisSpeeds               = new ChassisSpeeds(0, 0, setpoint.velocity + output);
        DifferentialDriveWheelSpeeds wheelSpeeds  = s_driveTrain.getKinematics().toWheelSpeeds(chassisSpeeds);

        System.out.println(chassisSpeeds.omegaRadiansPerSecond);
        s_driveTrain.setVelocityPID(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
      }
    );
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_driveTrain);

    // Configure additional PID options by calling `getController` here...
    this.getController().enableContinuousInput(-Math.PI, Math.PI);
    this.getController().setTolerance(Math.toRadians(0.5), 0.2);        // Velocity tolerance NOT IN M/S, but in radians/s

  } 

  //--------------------------------------------------------------------------------------------------
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.getController().atGoal();
  }
}
