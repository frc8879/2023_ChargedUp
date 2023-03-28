package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
    
    /*
    * Drive motor controller instances.
    */
    private final CANSparkMax driveLeftLead;
    private final CANSparkMax driveRightLead;
    private final CANSparkMax driveLeftFollow;
    private final CANSparkMax driveRightFollow;

    private final SparkMaxPIDController leftPIDController;
    private final SparkMaxPIDController rightPIDController;

    /*
    * Drivetrain Odometry  
    */
    private final WPI_Pigeon2 gyro;
    private final DifferentialDriveOdometry odometry;
    private double leftTarget;
    private double rightTarget;

    public DriveTrain() {
        driveLeftLead = new CANSparkMax(Constants.LEFT_LEAD_ID, MotorType.kBrushless);
        driveRightLead = new CANSparkMax(Constants.RIGHT_LEAD_ID, MotorType.kBrushless);
        driveLeftFollow = new CANSparkMax(Constants.LEFT_FOLLOW_ID, MotorType.kBrushless);
        driveRightFollow = new CANSparkMax(Constants.RIGHT_FOLLOW_ID, MotorType.kBrushless);

        /*
        * You will need to change some of these from false to true.
        * 
        * In the setDriveMotors method, comment out all but 1 of the 4 calls
        * to the set() methods. Push the joystick forward. Reverse the motor
        * if it is going the wrong way. Repeat for the other 3 motors.
        */
        driveLeftLead.setInverted(false);
        driveLeftFollow.setInverted(false);
        driveRightLead.setInverted(true);
        driveRightFollow.setInverted(true);

        driveLeftLead.setIdleMode(IdleMode.kBrake);
        driveLeftFollow.setIdleMode(IdleMode.kBrake);
        driveRightLead.setIdleMode(IdleMode.kBrake);
        driveRightFollow.setIdleMode(IdleMode.kBrake);

        // Set the second sparks on each side to follow the leader.
        // There is another note below on testing follow
        driveLeftFollow.follow(driveLeftLead);
        driveRightFollow.follow(driveRightLead);

        /*
        * @Hector
        * Instantiate PID controllers and set P gain. This value of P will need to be tuned. See instructions below.
        */
        leftPIDController = driveLeftLead.getPIDController();
        rightPIDController = driveRightLead.getPIDController();

        leftPIDController.setP(Constants.DRIVETRAIN_KP);
        rightPIDController.setP(Constants.DRIVETRAIN_KP);
        //leftPIDController.setFF(Constants.DRIVETRAIN_KF);
        //rightPIDController.setFF(Constants.DRIVETRAIN_KF);
        leftPIDController.setOutputRange(Constants.DRIVETRAIN_MIN_OUTPUT, Constants.DRIVETRAIN_MAX_OUTPUT);
        rightPIDController.setOutputRange(Constants.DRIVETRAIN_MIN_OUTPUT, Constants.DRIVETRAIN_MAX_OUTPUT);

        /*
        * Initialize the Odometry information
        */
        gyro = new WPI_Pigeon2(1);
        gyro.setYaw(0.0);
        RelativeEncoder leftEncoder = driveLeftLead.getEncoder();
        RelativeEncoder rightEncoder = driveRightLead.getEncoder();
        leftEncoder.setPosition(0.0);
        rightEncoder.setPosition(0.0);
        // Gearing is computed as the distance in meters traveled by 1 revolution of the NEO output shaft
        
        //double gearing = 1.0;
        leftEncoder.setPositionConversionFactor(Constants.DRIVETRAIN_GEARING);
        rightEncoder.setPositionConversionFactor(Constants.DRIVETRAIN_GEARING);
        odometry = new DifferentialDriveOdometry(
        gyro.getRotation2d(), 
        driveLeftLead.getEncoder().getPosition(),
        driveRightLead.getEncoder().getPosition());
    }

    /**
     * Calculate and set the power to apply to the left and right
     * drive motors.
     * 
     * @param forward Desired forward speed. Positive is forward.
     * @param turn    Desired turning speed. Positive is counter clockwise from
     *                above.
     */
    public void setVelocity(double forward, double turn, boolean highSpeed) {
        SmartDashboard.putNumber("drive forward speed (%)", forward);
        SmartDashboard.putNumber("drive turn speed (%)", turn);

        leftPIDController.setP(Constants.DRIVETRAIN_KP);
        rightPIDController.setP(Constants.DRIVETRAIN_KP);

        // Add Deadband
        if(Math.abs(forward) < 0.1){
            forward=0.0;
        }
        if(Math.abs(turn) < 0.1){
            turn=0.0;
        }

        forward = Math.copySign(forward * forward, forward);
        turn = Math.copySign(turn * turn, turn);

        /*
            * positive turn = counter clockwise, so the left side goes backwards
            */
        double left = forward - turn;
        double right = forward + turn;

        /*
            * Use the DifferentialDrive to compute chasis speeds
            */
        if(!highSpeed){
            left=left*0.5;
            right=right*0.5;
        }
        WheelSpeeds wheelSpeeds = DifferentialDrive.curvatureDriveIK(left, right, !highSpeed);

        SmartDashboard.putNumber("drive left speed (%)", wheelSpeeds.left);
        SmartDashboard.putNumber("drive right speed (%)", wheelSpeeds.right);

        // @Hector TODO: Validate that the spark2 motors are driving as followers. 
        // To do this, enable in teleop and drive gently forward. All of the motor controllers should blink green.
        // If only the lead motors are blinking green, uncomment these and comment out the "follow" method above.
        // Let me know if this isn't working so we can adjust the braking code.

        leftPIDController.setReference(left*Constants.DRIVETRAIN_MAX_RPM_LOAD, CANSparkMax.ControlType.kVelocity);
        rightPIDController.setReference(right*Constants.DRIVETRAIN_MAX_RPM_LOAD, CANSparkMax.ControlType.kVelocity);
    }   

    /**
     * Calculate and set the power to apply to the left and right
     * drive motors.
     * 
     * @param forward Desired forward speed. Positive is forward.
     * @param turn    Desired turning speed. Positive is counter clockwise from
     *                above.
     */
    public void setDriveMotors(double forward, double turn, boolean highSpeed) {
        SmartDashboard.putNumber("drive forward power (%)", forward);
        SmartDashboard.putNumber("drive turn power (%)", turn);

        // Add Deadband
        if(Math.abs(forward) < 0.1){
            forward=0.0;
        }
        if(Math.abs(turn) < 0.1){
            turn=0.0;
        }

        forward = Math.copySign(forward * forward, forward);
        turn = Math.copySign(turn * turn, turn);

        /*
        * positive turn = counter clockwise, so the left side goes backwards
        */
        double left = forward - turn;
        double right = forward + turn;

        /*
            * Use the DifferentialDrive to compute chasis speeds
            */
        if(!highSpeed){
            left=left*0.5;
            right=right*0.5;
        }
        WheelSpeeds wheelSpeeds = DifferentialDrive.curvatureDriveIK(left, right, !highSpeed);

        SmartDashboard.putNumber("drive left power (%)", wheelSpeeds.left);
        SmartDashboard.putNumber("drive right power (%)", wheelSpeeds.right);

        // @Hector TODO: Validate that the spark2 motors are driving as followers. 
        // To do this, enable in teleop and drive gently forward. All of the motor controllers should blink green.
        // If only the lead motors are blinking green, uncomment these and comment out the "follow" method above.
        // Let me know if this isn't working so we can adjust the braking code.

        driveLeftLead.set(left);
        //driveLeftSpark2.set(left);
        driveRightLead.set(right);
        //driveRightSpark2.set(right);
    }

    @Override
    public void periodic() {
    SmartDashboard.putNumber("Time (seconds)", Timer.getFPGATimestamp());
    /**
        * Update Odometry using information from the Pigeon and the motors
        * Then push that updated information to the dashboard
        */
        odometry.update(
        Rotation2d.fromDegrees(0), 
        driveLeftLead.getEncoder().getPosition(), 
        driveRightLead.getEncoder().getPosition());

        SmartDashboard.putNumber("LeftEncoder", driveLeftLead.getEncoder().getPosition());
        SmartDashboard.putNumber("RightEncoder", driveRightLead.getEncoder().getPosition());

        SmartDashboard.putNumber("x", odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("y", odometry.getPoseMeters().getX());

        SmartDashboard.putNumber("Pitch", gyro.getPitch());
        SmartDashboard.putNumber("Roll", gyro.getRoll());
        SmartDashboard.putNumber("Yaw", gyro.getYaw());
    }

    public Pose2d getPoseMeters() {
    return odometry.getPoseMeters();
    }

    public void setBrake() {
        leftTarget = driveLeftLead.getEncoder().getPosition();
        rightTarget = driveRightLead.getEncoder().getPosition();
        SmartDashboard.putNumber("LeftTarget", leftTarget);
        SmartDashboard.putNumber("RightTarget", rightTarget);
    }

    public void brake() {
        leftPIDController.setP(2.0);
        rightPIDController.setP(2.0);
        leftPIDController.setReference(leftTarget, CANSparkMax.ControlType.kPosition);
        rightPIDController.setReference(rightTarget, CANSparkMax.ControlType.kPosition);
    }

    public void resetPosition() {
        driveLeftLead.getEncoder().setPosition(0.0);
        driveRightLead.getEncoder().setPosition(0.0);
    }

    public double getDistance() {
        return (driveLeftLead.getEncoder().getPosition() + driveRightLead.getEncoder().getPosition()) / 2.0;
    }

}
