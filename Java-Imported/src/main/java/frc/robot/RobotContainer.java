package frc.robot;

import static frc.robot.Constants.JoyConstants.P_OI_JOY_LEFT;
import static frc.robot.Constants.JoyConstants.P_OI_JOY_MECH;
import static frc.robot.Constants.JoyConstants.P_OI_JOY_RIGHT;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.PowerCellConstants.E_SHOOT_POS;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunShooter;
import frc.robot.subsystems.BallMagazine;
import frc.robot.subsystems.BallShooter;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Lift;


public class RobotContainer{
    
    //--------------------------------------------------------------------------------------------------
    // Declare Robot Subsystems
    //private final DriveTrain    s_driveTrain;
    public final  BallShooter   s_ballShooter;
    //public final  BallIntake    s_ballIntake;
    public final  BallMagazine  s_ballMagazine;
    public final  Elevator      s_elevator;
    public final  Lift          s_lift;
    
    // Declare Commands
    private RunShooter spinUpClose;
    private RunShooter spinUpTrench;

    private RunIntake runIntake;

    // Declare JoyStick
    private Joystick leftJoy    = new Joystick(P_OI_JOY_LEFT);
    private Joystick rightJoy   = new Joystick(P_OI_JOY_RIGHT);            

    private Joystick mechJoy    = new Joystick(P_OI_JOY_MECH);

    // Declare Buttons
    private JoystickButton slowModeLeft = new JoystickButton(leftJoy, 1),
                            slowModeRight = new JoystickButton(rightJoy, 1),
                            intake = new JoystickButton(mechJoy, 7),
                            toggleShooterClose = new JoystickButton(mechJoy, 2),
                            toggleShooterTrench = new JoystickButton(mechJoy, 3),
                            feedBall = new JoystickButton(mechJoy, 8),
                            shiftBall = new JoystickButton(mechJoy, 5);

    private int pov = mechJoy.getPOV(0);


    //--------------------------------------------------------------------------------------------------
    // Constructor
    public RobotContainer(){
        //s_driveTrain    = new DriveTrain();
        s_ballShooter   = new BallShooter();
        //s_ballIntake    = new BallIntake();
        s_ballMagazine  = new BallMagazine();
        s_elevator      = new Elevator();
        s_lift          = new Lift();
        
        spinUpClose     = new RunShooter(E_SHOOT_POS.CLOSE, s_ballShooter, s_ballMagazine);
        spinUpTrench    = new RunShooter(E_SHOOT_POS.TRENCH, s_ballShooter, s_ballMagazine);
        //runIntake       = new RunIntake(s_ballIntake, s_ballMagazine);

        //Tank Drive
        /*s_driveTrain.setDefaultCommand(
            new TankDrive(
                () -> -leftJoy.getY(), 
                () -> rightJoy.getY(), 
                s_driveTrain)
        );*/

        //Configure default commands (for auton choosing), button bindings
        configureButtonBindings();
    }

    //--------------------------------------------------------------------------------------------------
    // Configuring Buttons

    private void configureButtonBindings(){
        //TODO: Configure button bindings
        /*slowModeRight.whenPressed(()    -> s_driveTrain.setSlow(true))
                                         .whenReleased(()   -> s_driveTrain.setSlow(false));

        slowModeLeft.whenPressed(()     -> s_driveTrain.setSlow(true))
                                        .whenReleased(()    -> s_driveTrain.setSlow(false));
        */
        // BallIntake Commands
        //intake.toggleWhenPressed(new RunIntake(s_ballIntake));
        
        // BallShooter Commands
        toggleShooterClose.toggleWhenPressed(spinUpClose).cancelWhenPressed(spinUpTrench);
        toggleShooterTrench.toggleWhenPressed(spinUpTrench).cancelWhenPressed(spinUpClose);
        
        //toggleShooterClose.whileActiveContinuous(() -> s_ballShooter.setShooterVelocityPID(50)).whenInactive(() -> s_ballShooter.stopShooter());

        // BallMagazine Commands
        //feedBall.and(new Trigger(() -> s_ballShooter.isFlywheelReady())).whenActive(() -> s_ballMagazine.feedBall());
        feedBall.whenPressed(() -> s_ballMagazine.feedBall()).whenReleased(() -> {
            s_ballMagazine.stopMagazine();
            s_ballMagazine.stopPreroller();
        });

        (new JoystickButton(mechJoy, 4)).whenActive(() -> s_elevator.eleLiftPercent(0.4)).whenInactive(() -> s_elevator.stall());

        (new JoystickButton(mechJoy, 1)).whileHeld(() -> s_lift.liftPercent(0.75)).whenReleased(() -> s_lift.stopMotor());

        //(new JoystickButton(mechJoy, 6)).whenPressed(() -> s_ballIntake.extend()).whenReleased(() -> s_ballIntake.contract());

        (new JoystickButton(mechJoy, 10)).whenPressed(() -> s_elevator.zeroEncoder());

        //intake.whileHeld(() -> s_ballMagazine.stopPreroller()).whileHeld(runIntake);
        shiftBall.whileHeld(() -> s_ballMagazine.shiftBall()).whenReleased(() -> s_ballMagazine.stopMagazine());

        // ElevatorLift Commands
        /*(new JoystickButton(leftJoy, 5)).whileActiveOnce(new ElevatorMax(s_elevator));
        
        (new JoystickButton(leftJoy, 6)).whenPressed(   ()-> s_elevator.lowerElevator(C_ELEVATOR_lower_slow_VOLT))
                                        .whenReleased(  ()-> s_elevator.stall());

        (new JoystickButton(leftJoy, 7)).whenPressed(   ()-> s_elevator.lowerElevator(C_ELEVATOR_lower_fast_VOLT))
                                        .whenReleased(  new LiftRobot(s_lift, s_elevator)); */
    }

    public Command getAutonomousCommand(String m_autoSelected){
        /*s_driveTrain.zeroHeading();
        s_driveTrain.resetOdometry(new Pose2d(0.0, 0.0, new Rotation2d(0.0)));
        
        return new ProfiledTurnTo(-45, s_driveTrain);*/
        return null;

        /*switch (m_autoSelected) {
            //case "TurnTo":
                //return new TurnTo(SmartDashboard.getNumber("Degrees", 0.0), s_driveTrain);
            case "PushBotAndReturnBad":
                return (new DriveDistanceBad(0.1, 1.0, s_driveTrain)).andThen(new DriveDistanceBad(-1.0, -0.4, s_driveTrain));
            case "PushBotAndReturnFF":
                return (new DriveDistance(0.1, 0.1, s_driveTrain)).andThen(new DriveDistance(0.1, -0.2, s_driveTrain));
            default:
                return null;
        }*/

    }

}