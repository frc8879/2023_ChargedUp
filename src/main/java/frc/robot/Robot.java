// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  /*
   * Autonomous selection options.
   */
  private static final String kNothingAuto = "do nothing";
  private static final String kConeAuto = "cone";
  private static final String kCubeAuto = "cube";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /*
   * Drive motor controller instances.
   * 
   * Change the id's to match your robot.
   * Change kBrushed to kBrushless if you are using NEO's.
   * Use the appropriate other class if you are using different controllers.
   */
  CANSparkMax driveLeftSpark = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax driveRightSpark = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax driveLeftSpark2 = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax driveRightSpark2 = new CANSparkMax(4, MotorType.kBrushless);

  /*
   * Drivetrain Odometry
   * Add the following components
   * DifferentialDriveOdometry - This is used for tracking the robot on the field.
   * Field2d - This is a dashboard element to view the robot on the field
   * WPI_Pigeon2 - This is the gyro that lets us know which way the robot is facing
   */
  private Field2d field;
  private WPI_Pigeon2 gyro;
  private DifferentialDriveOdometry odometry;

  /*
   * Mechanism motor controller instances.
   * 
   * Like the drive motors, set the CAN id's to match your robot or use different
   * motor controller classses (TalonFX, TalonSRX, Spark, VictorSP) to match your
   * robot.
   * 
   * The arm is a NEO on Everybud.
   * The intake is a NEO 550 on Everybud.
   */
  CANSparkMax arm = new CANSparkMax(5, MotorType.kBrushless);
  CANSparkMax intake = new CANSparkMax(6, MotorType.kBrushless);

  /**
   * The starter code uses the most generic joystick class.
   * 
   * The reveal video was filmed using a logitech gamepad set to
   * directinput mode (switch set to D on the bottom). You may want
   * to use the XBoxController class with the gamepad set to XInput
   * mode (switch set to X on the bottom) or a different controller
   * that you feel is more comfortable.
   */
  PS4Controller driverPS4 = new PS4Controller(0);
  PS4Controller coDriverPS4 = new PS4Controller(1);

  /*
   * Magic numbers. Use these to adjust settings.
   */

  /**
   * How many amps the arm motor can use.
   */
  static final int ARM_CURRENT_LIMIT_A = 20;

  /**
   * Percent output to run the arm up/down at
   */
  static final double ARM_OUTPUT_POWER = 0.4;

  /**
   * Arm details
   */
  double armTargetRotations = 0.0;

  /**
   * How many amps the intake can use while picking up
   */
  static final int INTAKE_CURRENT_LIMIT_A = 25;

  /**
   * How many amps the intake can use while holding
   */
  static final int INTAKE_HOLD_CURRENT_LIMIT_A = 5;

  /**
   * Percent output for intaking
   */
  static final double INTAKE_OUTPUT_POWER = 1.0;

  /**
   * Percent output for holding
   */
  static final double INTAKE_HOLD_POWER = 0.07;

  /**
   * Time to extend or retract arm in auto
   */
  static final double ARM_EXTEND_TIME_S = 2.0;

  /**
   * Time to throw game piece in auto
   */
  static final double AUTO_THROW_TIME_S = 0.375;

  /**
   * Time to drive back in auto
   */
  static final double AUTO_DRIVE_TIME = 6.0;

  /**
   * Speed to drive backwards in auto
   */
  static final double AUTO_DRIVE_SPEED = -0.25;

  /**
   * This method is run once when the robot is first started up.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("do nothing", kNothingAuto);
    m_chooser.addOption("cone and mobility", kConeAuto);
    m_chooser.addOption("cube and mobility", kCubeAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    /*
     * You will need to change some of these from false to true.
     * 
     * In the setDriveMotors method, comment out all but 1 of the 4 calls
     * to the set() methods. Push the joystick forward. Reverse the motor
     * if it is going the wrong way. Repeat for the other 3 motors.
     */
    driveLeftSpark.setInverted(false);
    driveLeftSpark2.setInverted(false);
    driveRightSpark.setInverted(true);
    driveRightSpark2.setInverted(true);

    driveLeftSpark.setIdleMode(IdleMode.kBrake);
    driveLeftSpark2.setIdleMode(IdleMode.kBrake);
    driveRightSpark.setIdleMode(IdleMode.kBrake);
    driveRightSpark2.setIdleMode(IdleMode.kBrake);
    

    /*
     * Set the arm and intake to brake mode to help hold position.
     * If either one is reversed, change that here too. Arm out is defined
     * as positive, arm in is negative.
     */
    arm.setInverted(true);
    arm.setIdleMode(IdleMode.kBrake);
    arm.setSmartCurrentLimit(ARM_CURRENT_LIMIT_A);
    intake.setInverted(false);
    intake.setIdleMode(IdleMode.kBrake);

    /*
     * Initialize the Odometry information
     */
    field = new Field2d();
    gyro = new WPI_Pigeon2(1);
    gyro.setYaw(0.0);
    RelativeEncoder leftEncoder = driveLeftSpark.getEncoder();
    RelativeEncoder rightEncoder = driveRightSpark.getEncoder();
    leftEncoder.setPosition(0.0);
    rightEncoder.setPosition(0.0);
    // Gearing is computed as the distance in meters traveled by 1 revolution of the NEO output shaft
    double gearing = 0.05768;
    //double gearing = 1.0;
    leftEncoder.setPositionConversionFactor(gearing);
    rightEncoder.setPositionConversionFactor(gearing);
    odometry = new DifferentialDriveOdometry(
      gyro.getRotation2d(), 
      driveLeftSpark.getEncoder().getPosition(),
      driveRightSpark.getEncoder().getPosition());
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

    // see note above in robotInit about commenting these out one by one to set
    // directions.
    //driveLeftSpark.set(wheelSpeeds.left);
    //driveLeftSpark2.set(wheelSpeeds.left);
    //driveRightSpark.set(wheelSpeeds.right);
    //driveRightSpark2.set(wheelSpeeds.right);

    
    driveLeftSpark.set(left);
    driveLeftSpark2.set(left);
    driveRightSpark.set(right);
    driveRightSpark2.set(right);
  }

  /**
   * Set the arm output power. Positive is out, negative is in.
   * 
   * @param percent
   */
  public void setArmMotor(double percent) {
    arm.set(percent);
    SmartDashboard.putNumber("arm power (%)", percent);
    SmartDashboard.putNumber("arm motor current (amps)", arm.getOutputCurrent());
    SmartDashboard.putNumber("arm motor temperature (C)", arm.getMotorTemperature());
  }

  /**
   * Set the arm output power.
   * 
   * @param percent desired speed
   * @param amps current limit
   */
  public void setIntakeMotor(double percent, int amps) {
    intake.set(percent);
    intake.setSmartCurrentLimit(amps);
    SmartDashboard.putNumber("intake power (%)", percent);
    SmartDashboard.putNumber("intake motor current (amps)", intake.getOutputCurrent());
    SmartDashboard.putNumber("intake motor temperature (C)", intake.getMotorTemperature());
  }

  /**
   * This method is called every 20 ms, no matter the mode. It runs after
   * the autonomous and teleop specific period methods.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Time (seconds)", Timer.getFPGATimestamp());
  /**
    * Update Odometry using information from the Pigeon and the motors
    * Then push that updated information to the dashboard
    */
    odometry.update(
      gyro.getRotation2d(), 
      driveLeftSpark.getEncoder().getPosition(), 
      driveRightSpark.getEncoder().getPosition());
    field.setRobotPose(odometry.getPoseMeters());

    SmartDashboard.putNumber("LeftEncoder", driveLeftSpark.getEncoder().getPosition());
    SmartDashboard.putNumber("RightEncoder", driveRightSpark.getEncoder().getPosition());

    SmartDashboard.putNumber("x", odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("y", odometry.getPoseMeters().getX());
  }

  double autonomousStartTime;
  double autonomousIntakePower;

  @Override
  public void autonomousInit() {

    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);

    if (m_autoSelected == kConeAuto) {
      autonomousIntakePower = INTAKE_OUTPUT_POWER;
    } else if (m_autoSelected == kCubeAuto) {
      autonomousIntakePower = -INTAKE_OUTPUT_POWER;
    }

    autonomousStartTime = Timer.getFPGATimestamp();
  }

  @Override
  public void autonomousPeriodic() {
    if (m_autoSelected == kNothingAuto) {
      setArmMotor(0.0);
      setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(0.0, 0.0, false);
      return;
    }

    double timeElapsed = Timer.getFPGATimestamp() - autonomousStartTime;

    if (timeElapsed < ARM_EXTEND_TIME_S) {
      setArmMotor(ARM_OUTPUT_POWER);
      setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(0.0, 0.0, false);
    } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S) {
      setArmMotor(0.0);
      setIntakeMotor(autonomousIntakePower, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(0.0, 0.0, false);
    } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S + ARM_EXTEND_TIME_S) {
      setArmMotor(-ARM_OUTPUT_POWER);
      setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(0.0, 0.0, false);
    } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S + ARM_EXTEND_TIME_S + AUTO_DRIVE_TIME) {
      setArmMotor(0.0);
      setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(AUTO_DRIVE_SPEED, 0.0, false);
    } else {
      setArmMotor(0.0);
      setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(0.0, 0.0, false);
    }
  }

  /**
   * Used to remember the last game piece picked up to apply some holding power.
   */
  static final int CONE = 1;
  static final int CUBE = 2;
  static final int NOTHING = 3;
  int lastGamePiece;

  @Override
  public void teleopInit() {
    lastGamePiece = NOTHING;
  }

  @Override
  public void teleopPeriodic() {

    // TODO identify the Rotations for low, mid, high, positions, etc.
    // TODO make a member variable for the current target Rotations
    double armPower;
    // TODO check buttons to set current state of target rotation
    if (coDriverPS4.getRawButton(7)) {
      // lower the arm
      armPower = -ARM_OUTPUT_POWER;
    } else if (coDriverPS4.getRawButton(5)) {
      // raise the arm
      armPower = ARM_OUTPUT_POWER;
    } else {
      // do nothing and let it sit where it is
      armPower = 0.0;
    }
    // TODO instead of setting the motor, set the PID reference to the armPositionRotation
    setArmMotor(armPower);
  
    double intakePower;
    int intakeAmps;
    if (coDriverPS4.getRawButton(8)) {
      // cube in or cone out
      intakePower = INTAKE_OUTPUT_POWER;
      intakeAmps = INTAKE_CURRENT_LIMIT_A;
      lastGamePiece = CUBE;
    } else if (coDriverPS4.getRawButton(6)) {
      // cone in or cube out
      intakePower = -INTAKE_OUTPUT_POWER;
      intakeAmps = INTAKE_CURRENT_LIMIT_A;
      lastGamePiece = CONE;
    } else if (lastGamePiece == CUBE) {
      intakePower = INTAKE_HOLD_POWER;
      intakeAmps = INTAKE_HOLD_CURRENT_LIMIT_A;
    } else if (lastGamePiece == CONE) {
      intakePower = -INTAKE_HOLD_POWER;
      intakeAmps = INTAKE_HOLD_CURRENT_LIMIT_A;
    } else {
      intakePower = 0.0;
      intakeAmps = 0;
    }
    setIntakeMotor(intakePower, intakeAmps);

    /*
     * Negative signs here because the values from the analog sticks are backwards
     * from what we want. Forward returns a negative when we want it positive.
     */

     // Add a high/low gear switch here
    setDriveMotors(-driverPS4.getRawAxis(1), -driverPS4.getRawAxis(2), driverPS4.getL1Button());
  }

}
