package frc.robot;

//Create a new "constants" class for each mech
public final class Constants{
    
    
    //Constants for the DriveTrain
    public static final class DriveConstants{
        public static final int P_DRIVE_LEFT_vicSPX_1 = 0,
                                        P_DRIVE_RIGHT_vicSPX_1 = 2,
                                        P_DRIVE_LEFT_vicSPX_2 = 1,
                                        P_DRIVE_RIGHT_vicSPX_2 = 3;
    }

    public static final class IntakeConstants{
        public static final int P_SHOOTER_VICTOR_1 = 0;
        public static final int P_SHOOTER_VICTOR_2 = 1;
    }

    //Constants for JoySticks
    public static final class JoyConstants{
        public static final int P_OI_JOY_LEFT = 0,
                                        P_OI_JOY_RIGHT = 2;
    }
}