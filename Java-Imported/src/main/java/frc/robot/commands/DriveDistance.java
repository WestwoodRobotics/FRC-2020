/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.DriveTrain;

import static frc.robot.Constants.DriveConstants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class DriveDistance extends ProfiledPIDCommand {
  /**
   * Creates a new DriveDistance.
   */
  public DriveDistance(double maxVel, double meters, DriveTrain s_driveTrain) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            C_kP_DIST, C_kI_DIST, C_kD_DIST,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(maxVel, 1)),
        // This should return the measurement
        () -> s_driveTrain.getAverageEncoderMeters(),
        // This should return the goal (can also be a constant)
        () -> new TrapezoidProfile.State(meters, 0.0),
        // This uses the output
        (output, setpoint) -> {
          // Use the output (and setpoint, if desired) here
          ChassisSpeeds chassisSpeeds = new ChassisSpeeds(setpoint.velocity + output, 0.0, 0.0);
          DifferentialDriveWheelSpeeds wheelSpeeds = s_driveTrain.getKinematics().toWheelSpeeds(chassisSpeeds);

          s_driveTrain.setVelocityPID(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(s_driveTrain);

    this.getController().disableContinuousInput();
    this.getController().setTolerance(0.1, 0.5);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.getController().atGoal();
  }
}
