/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;

import java.util.List;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new DriveTrain.
   */

  //--------------------------------------------------------------------------------------------------
  //Variables/Features of Drive Train
  private final WPI_TalonFX leftMaster  = new WPI_TalonFX(P_DRIVE_LEFT_master_talFX),
                                    rightMaster = new WPI_TalonFX(P_DRIVE_RIGHT_master_talFX);

  private final WPI_TalonFX leftFollow  = new WPI_TalonFX(P_DRIVE_LEFT_follow_talFX),
                            rightFollow = new WPI_TalonFX(P_DRIVE_RIGHT_follow_talFX);
  
                            
  /*private WPI_VictorSPX leftMaster  = new WPI_VictorSPX(P_DRIVE_LEFT_master_talFX),
                      rightMaster = new WPI_VictorSPX(P_DRIVE_RIGHT_master_talFX);

  private WPI_VictorSPX leftFollow  = new WPI_VictorSPX(P_DRIVE_LEFT_follow_talFX),
                      rightFollow = new WPI_VictorSPX(P_DRIVE_RIGHT_follow_talFX);*/
  

  private final PIDController leftVelPID = new PIDController(C_kP_LEFT, C_kI_LEFT, C_kD_LEFT);
  private final PIDController rightVelPID = new PIDController(C_kP_RIGHT, C_kI_RIGHT, C_kD_RIGHT);

  private final DifferentialDrive drive = new DifferentialDrive(leftMaster, rightMaster);
 
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(C_kS, C_kV, C_kA);
  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(C_TRACK_WIDTH_METERS);
  
  private final DifferentialDriveOdometry odometry;

  private boolean slowMode = false;

  private AHRS imu = new AHRS();

  /*insert encoders*/

  //--------------------------------------------------------------------------------------------------
  // Constructor
  public DriveTrain() {
    //leftMaster.setInverted(true);
    //leftFollow.setInverted(true);
    rightMaster.setInverted(true);
    rightFollow.setInverted(true);

    leftMaster.setSensorPhase(true);
    rightMaster.setSensorPhase(true);

    leftFollow.follow(leftMaster);
    rightFollow.follow(rightMaster);
    
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(this.getHeadingDegrees()));
    this.resetOdometry(new Pose2d(0.0, 0.0, new Rotation2d(0.0)));

    drive.setSafetyEnabled(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("X", odometry.getPoseMeters().getTranslation().getX());
    SmartDashboard.putNumber("Y", odometry.getPoseMeters().getTranslation().getY());
    SmartDashboard.putNumber("Heading", odometry.getPoseMeters().getRotation().getDegrees());
    
    SmartDashboard.putNumber("Left Encoder", this.leftEncoderGetMeters());
    SmartDashboard.putNumber("Right Encoder", this.rightEncoderGetMeters());

    odometry.update(
                Rotation2d.fromDegrees(this.getHeadingDegrees()), 
                this.leftEncoderGetMeters(), 
                this.rightEncoderGetMeters()
              );
  }

  //--------------------------------------------------------------------------------------------------
  // Drive wheels (percent) [-1.0, 1.0]

  public void driveWheelsPercent(double leftPercent, double rightPercent){
    drive.tankDrive(leftPercent, rightPercent);
  }

  public void driveWheelsVolts(double leftVolts, double rightVolts){
    leftMaster.setVoltage(leftVolts);
    rightMaster.setVoltage(rightVolts);
    drive.feed();
  }
  
  public void setVelocityPID(double leftMetersPerSec, double rightMetersPerSec){
    double leftVolts = 0.0;
    double rightVolts = 0.0;

    DifferentialDriveWheelSpeeds wheelSpeeds = this.getWheelSpeeds();

    leftVolts   += feedforward.calculate(leftMetersPerSec);
    rightVolts  += feedforward.calculate(rightMetersPerSec);

    leftVolts   += leftVelPID.calculate(wheelSpeeds.leftMetersPerSecond, leftMetersPerSec);
    rightVolts  += rightVelPID.calculate(wheelSpeeds.rightMetersPerSecond, rightMetersPerSec);

    //System.out.println(leftVolts + ", " + rightVolts);

    this.driveWheelsVolts(leftVolts, rightVolts);
  }

  // Converts ticks/decS to m/s
  public double falconVelToMetersPerSec(double ticksPerDecasec){
    double metersPerSec = ticksPerDecasec*10.0*Math.PI*C_WHEEL_DIAMETER_METERS/C_DRIVE_EPR;
    return metersPerSec;
  }
    
  //Slow mode
  public void     setSlow(boolean slowMode)       {this.slowMode = slowMode;}
  public boolean  getSlow()                       {return slowMode;}
  
  public void   setMaxOutput(double maxOutput)  {drive.setMaxOutput(maxOutput);}
  public void   stopWheels()                    {drive.stopMotor();}

  //--------------------------------------------------------------------------------------------------
  // Methods for Odometry/Trejectory

  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose){
    zeroLeftEncoder();
    zeroRightEncoder();

    odometry.resetPosition(pose, Rotation2d.fromDegrees(this.getHeadingDegrees()));
  }

  public Command getTrajectoryCommand(){
    resetOdometry(new Pose2d(0.0, 0.0, new Rotation2d(0.0)));

    DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                                                                      this.getFeedForward(), 
                                                                      this.getKinematics(), 
                                                                      10.0
                                                                    );
    TrajectoryConfig config = new TrajectoryConfig(C_maxVel, C_maxAccel).setKinematics(this.getKinematics()).addConstraint(autoVoltageConstraint);
    
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0.0, 0.0, new Rotation2d(0.0)), 
      List.of(                                                                            // List of points that we want to hit on the field
        new Translation2d(1.0, 0.0),
        new Translation2d(2.0, 0.0),
        new Translation2d(2.0, -1.0),
        new Translation2d(1.0, -1.0),
        new Translation2d(1.0, -2.0)
      ), 
      new Pose2d(2.0, -2.0, new Rotation2d(90.0)),
      config
    );
    
    //System.out.println(exampleTrajectory.getStates());
    RamseteCommand ramseteCommand = new RamseteCommand(
                                          exampleTrajectory, 
                                          this::getPose, 
                                          new RamseteController(C_kB_RAMSETE, C_kZeta_RAMSETE), 
                                          this.getFeedForward(), this.getKinematics(), 
                                          this::getWheelSpeeds, 
                                          this.getController("l"), this.getController("r"), 
                                          this::driveWheelsVolts, this
                                        );

    return ramseteCommand;
  }

  //-------------------------------------------------------------
      //Turning
  public double getHeadingDegrees()         {return -imu.pidGet();}
  public double getHeadingRadians()         {return Math.toRadians(this.getHeadingDegrees());}
  public double getTurnRate()               {return imu.getRate();} 
  public void   zeroHeading()               {imu.reset();}

  //-------------------------------------------------------------
      //Encoders
  public double leftEncoderGetTicks()       {return leftMaster.getSelectedSensorPosition();}
  public double rightEncoderGetTicks()      {return rightMaster.getSelectedSensorPosition();}
  public double leftEncoderGetMeters()      {return ticksToMeters(this.leftEncoderGetTicks());}
  public double rightEncoderGetMeters()     {return ticksToMeters(this.rightEncoderGetTicks());}
  public double getAverageEncoderTicks()    {return (leftEncoderGetTicks() + rightEncoderGetTicks())/2.0;}
  public double getAverageEncoderMeters()   {return (leftEncoderGetMeters() + rightEncoderGetMeters())/2.0;}
  public void   zeroLeftEncoder()           {leftMaster.setSelectedSensorPosition(0);}
  public void   zeroRightEncoder()          {rightMaster.setSelectedSensorPosition(0);}
 
//-------------------------------------------------------------
  public DifferentialDriveKinematics getKinematics()  {return this.kinematics;}
  public SimpleMotorFeedforward      getFeedForward() {return this.feedforward;}

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    DifferentialDriveWheelSpeeds speeds = new DifferentialDriveWheelSpeeds(
                                                this.falconVelToMetersPerSec(leftMaster.getSelectedSensorVelocity()), 
                                                this.falconVelToMetersPerSec(rightMaster.getSelectedSensorVelocity())
                                              );
    SmartDashboard.putNumber("left speed", speeds.leftMetersPerSecond);
    SmartDashboard.putNumber("right speed", speeds.rightMetersPerSecond);
    return speeds;
  }

  public PIDController getController(String leftOrRight){
    if(leftOrRight.equalsIgnoreCase("left") || leftOrRight.equalsIgnoreCase("l")){
      return this.leftVelPID;
    }
    else if(leftOrRight.equalsIgnoreCase("right") || leftOrRight.equalsIgnoreCase("r")){
      return this.rightVelPID;
    }
    else{
      throw new IllegalArgumentException("Invalid argument for getController()");
    }
  }
  //--------------------------------------------------------------------------------------------------

  public void turnRate(double rt){
    drive.curvatureDrive(0, rt, true);
  }

  //--------------------------------------------------------------------------------------------------
}
