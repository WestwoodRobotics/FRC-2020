package frc.robot;

import static frc.robot.Constants.JoyConstants.P_OI_JOY_LEFT;
import static frc.robot.Constants.JoyConstants.P_OI_JOY_RIGHT;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ProfiledTurnTo;
import frc.robot.commands.TankDrive;
import frc.robot.commands.DriveDistTrapezoidProf;
import frc.robot.subsystems.BallShooter;
import frc.robot.subsystems.DriveTrain;

public class RobotContainer{
    //--------------------------------------------------------------------------------------------------
    // Declare Robot Subsystems
    private final DriveTrain s_driveTrain;
    public final BallShooter s_ballShooter;

    private ProfiledTurnTo a_turn2;

    private Joystick rightJoy  = new Joystick(P_OI_JOY_RIGHT);            // Declaring Joysticks
    private Joystick leftJoy = new Joystick(P_OI_JOY_LEFT);

    //--------------------------------------------------------------------------------------------------
    // Constructor
    public RobotContainer(){
        s_driveTrain = new DriveTrain();
        s_ballShooter = new BallShooter();
        a_turn2 = new ProfiledTurnTo(90, s_driveTrain);
        //s_ballShooter = new BallShooter();

        //Tank Drive
        s_driveTrain.setDefaultCommand(
            new TankDrive(
                () -> -rightJoy.getY(), 
                () -> leftJoy.getY(), 
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
        
        //(new JoystickButton(rightJoy, 3)).whileHeld(() -> SmartDashboard.putNumber("left Encoder: ", s_driveTrain.leftEncoderGet()));

        // BallShooter Commands
        (new JoystickButton(leftJoy, 3)).whenPressed(() -> s_ballShooter.intakeBall()).whenReleased(() -> s_ballShooter.stopBall());
        (new JoystickButton(leftJoy, 4)).whenPressed(() -> s_ballShooter.shootBall()).whenReleased(() -> s_ballShooter.stopBall());

    }

    public Command getAutonomousCommand(){
        return new DriveDistTrapezoidProf(10.0, s_driveTrain);
    }

    public void resetGyro(){
        s_driveTrain.zeroHeading();
    }
}