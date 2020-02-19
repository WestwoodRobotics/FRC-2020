package frc.robot;

import static frc.robot.Constants.JoyConstants.P_OI_JOY_LEFT;
import static frc.robot.Constants.JoyConstants.P_OI_JOY_RIGHT;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ProfiledTurnTo;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.BallShooter;
import frc.robot.subsystems.DriveTrain;

public class RobotContainer{
    //--------------------------------------------------------------------------------------------------
    // Declare Robot Subsystems
    private final DriveTrain s_driveTrain;
    public final BallShooter s_ballShooter;

    private ProfiledTurnTo a_turn2;

    private Joystick leftJoy = new Joystick(P_OI_JOY_LEFT);
    private Joystick rightJoy  = new Joystick(P_OI_JOY_RIGHT);            // Declaring Joysticks

    //--------------------------------------------------------------------------------------------------
    // Constructor
    public RobotContainer(){
        s_driveTrain = new DriveTrain();
        s_ballShooter = new BallShooter();
        a_turn2 = new ProfiledTurnTo(90.0, s_driveTrain);
        //s_ballShooter = new BallShooter();

        //Tank Drive
        s_driveTrain.setDefaultCommand(
            new TankDrive(
                () -> -leftJoy.getY(), 
                () -> rightJoy.getY(), 
                s_driveTrain)
        );

        //s_driveTrain.zeroLeftEncoder();
        //s_driveTrain.zeroRightEncoder();

        //new RunCommand(() -> s_driveTrain.getHeading());

        //Configure default commands (for auton choosing), button bindings
        configureButtonBindings();
    }

    private void configureButtonBindings(){
        //Configure button bindings
        (new JoystickButton(rightJoy, 1)).whenPressed(() -> s_driveTrain.setSlow(true)).whenReleased(() -> s_driveTrain.setSlow(false));
        (new JoystickButton(leftJoy, 1)).whenPressed(() -> s_driveTrain.setSlow(true)).whenReleased(() -> s_driveTrain.setSlow(false));

        // BallShooter Commands
        (new JoystickButton(leftJoy, 3)).whenPressed(() -> s_ballShooter.intakeBall()).whenReleased(() -> s_ballShooter.stopBall());
        (new JoystickButton(leftJoy, 4)).whenPressed(() -> s_ballShooter.shootBall()).whenReleased(() -> s_ballShooter.stopBall());

    }

    public Command getAutonomousCommand(){
        s_driveTrain.zeroHeading();
        s_driveTrain.resetOdometry(new Pose2d(0.0, 0.0, new Rotation2d(0.0)));

        //return a_turn2;
        return s_driveTrain.getTrajectoryCommand();
    }
}