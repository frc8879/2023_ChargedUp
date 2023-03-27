package frc.robot;

public class Constants {

    /*
     * IDs
     */
    public static final int LEFT_LEAD_ID = 1;
    public static final int RIGHT_LEAD_ID = 2;
    public static final int LEFT_FOLLOW_ID = 3;
    public static final int RIGHT_FOLLOW_ID = 4;

    public static final int ARM_ID = 5;
    public static final int INTAKE_ID = 6;

    /*
    * DriveTrain constants
    */
    public static final double DRIVETRAIN_GEARING = 0.05768;
    public static final double DRIVETRAIN_KP = 0.0001;
    public static final double DRIVETRAIN_KF = 0.000015;
    public static final double DRIVETRAIN_MAX_OUTPUT = 1.0;
    public static final double DRIVETRAIN_MIN_OUTPUT = -1.0;
    public static final double DRIVETRAIN_MAX_RPM_FREE_SPEED = 10000;
    public static final double DRIVETRAIN_MAX_RPM_LOAD = DRIVETRAIN_MAX_RPM_FREE_SPEED * 0.8;

    /*
    * Arm constants
    */
    public static final int ARM_CURRENT_LIMIT_A = 30;
    public static final double ARM_OUTPUT_POWER = 0.2;

    public static final double ARM_POSITION_STOWED = 0.0;

    /*
    * Intake constants
    */
    public static final int INTAKE_CURRENT_LIMIT_A = 25;
    public static final int INTAKE_HOLD_CURRENT_LIMIT_A = 5;
    public static final double INTAKE_OUTPUT_POWER = 0.8;
    public static final double INTAKE_HOLD_POWER = 0.04;

}
