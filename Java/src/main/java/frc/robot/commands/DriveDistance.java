/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class DriveDistance extends ProfiledPIDCommand {
  /**
   * Creates a new DriveDistance.
   */
  public DriveDistance(double meters, DriveTrain s_driveTrain) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            C_kP_dist, C_kI_dist, C_kD_dist,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(1, 8)
          ),

        // This should return the measurement
        () -> s_driveTrain.ticksToMeters(s_driveTrain.leftEncoderGetTicks()),
        // This should return the goal (can also be a constant)
        () -> 10.0,
        // This uses the output
        (output, setpoint) -> {
          // Use the output (and setpoint, if desired) here
          //System.out.println(setpo);
          s_driveTrain.setVelocityPID(setpoint.velocity, setpoint.velocity);
          //System.out.println("vel: " + setpoint.velocity);
        },
        s_driveTrain
    );

    s_driveTrain.zeroLeftEncoder();
    s_driveTrain.zeroRightEncoder();

    this.getController().disableContinuousInput();
    this.getController().setTolerance(.2);

    //System.out.println(meters + s_driveTrain.ticksToMeters(s_driveTrain.leftEncoderGetTicks()));

    addRequirements(s_driveTrain);

    //System.out.println("\n\n\nGoal: " + this.getController().getGoal().position + "\n\n\n");

    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //System.out.println("finished");
    return this.getController().atSetpoint();
  }
}
