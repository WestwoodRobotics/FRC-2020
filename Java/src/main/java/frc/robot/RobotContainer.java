package frc.robot;

import static frc.robot.Constants.JoyConstants.P_OI_JOY_LEFT;
import static frc.robot.Constants.JoyConstants.P_OI_JOY_RIGHT;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ElevatorMax;
import frc.robot.commands.ProfiledTurnTo;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.BallIntake;
import frc.robot.subsystems.BallShooter;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Lift;

public class RobotContainer{
    
    //--------------------------------------------------------------------------------------------------
        // Declare Robot Subsystems
    private final DriveTrain    s_driveTrain;
    public final  BallShooter   s_ballShooter;
    public final  BallIntake    s_ballIntake;
    public final  Elevator      s_elevator;
    public final  Lift          s_lift;
    
        // Declare Commands
    private ProfiledTurnTo      a_turn2;

        // Declare JoyStick
    private Joystick leftJoy    = new Joystick(P_OI_JOY_LEFT);
    private Joystick rightJoy   = new Joystick(P_OI_JOY_RIGHT);            

    //--------------------------------------------------------------------------------------------------
    // Constructor
    public RobotContainer(){
        s_driveTrain    = new DriveTrain();
        s_ballShooter   = new BallShooter();
        s_ballIntake    = new BallIntake();
        s_elevator      = new Elevator();
        s_lift          = new Lift();
        a_turn2         = new ProfiledTurnTo(90.0, s_driveTrain);
     
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

    //--------------------------------------------------------------------------------------------------
    // Configuring Buttons

    private void configureButtonBindings(){
        //Configure button bindings
        (new JoystickButton(rightJoy, 1)).whenPressed(()    -> s_driveTrain.setSlow(true))
                                         .whenReleased(()   -> s_driveTrain.setSlow(false));

        (new JoystickButton(leftJoy, 1)).whenPressed(()     -> s_driveTrain.setSlow(true))
                                        .whenReleased(()    -> s_driveTrain.setSlow(false));

        // BallIntake Commands
        (new JoystickButton(leftJoy, 3)).whenPressed(()     -> s_ballIntake.intakeBall())
                                        .whenReleased(()    -> s_ballIntake.stopBall());
        
        // BallShooter Commands
        (new JoystickButton(leftJoy, 4)).whenPressed(()     -> s_ballShooter.shootBall())
                                        .whenReleased(()    -> s_ballShooter.stopBall());

        // Elevator Commands
        (new JoystickButton(leftJoy, 5)).whileActiveOnce(new ElevatorMax(s_elevator));
        
        // Lift Commands
        (new JoystickButton(leftJoy, 6)).whenPressed(()     -> s_lift.liftPercentage(1))      //TODO: change speed*******************
                                        .whenReleased(()    -> s_lift.stopMotor());

    }

    //--------------------------------------------------------------------------------------------------
    // Autonomous Commands

    public Command getAutonomousCommand(){
        s_driveTrain.zeroHeading();
        s_driveTrain.resetOdometry(new Pose2d(0.0, 0.0, new Rotation2d(0.0)));

        //s_ballShooter.intakeBall();                       NEED TO time how long to intake balls and then stop 
        //s_ballShooter.stopBall();
        //s_ballShooter.shootBall();

        //return a_turn2;
        return s_driveTrain.getTrajectoryCommand()
                           .andThen(new ProfiledTurnTo(90, s_driveTrain));



    }
}