// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.AutoRoutine;
import frc.robot.autonomous.SanAntonioAuto;
import frc.robot.subsystems.ArmMode;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;

public class Robot extends TimedRobot {
  /*
   * Autonomous selection options.
   */
  private static final String kNothingAuto = "do nothing";
  private static final String sanAntonioAuto = "San Antonio auto";
  private static final String kCubeAuto = "cube";
  private String autoSelected;
  private final SendableChooser<String> autoChooser = new SendableChooser<>();
  private AutoRoutine autoRoutine;

  /* 
   * Instantiate the subsystems
   */
  private final DriveTrain driveTrain = new DriveTrain();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final Intake intake = new Intake();
  private Field2d field;

  /**
   * Instantiate the PS4 Controllers
   */
  PS4Controller driverPS4 = new PS4Controller(0);
  PS4Controller coDriverPS4 = new PS4Controller(1);

  /**
   * This method is run once when the robot is first started up.
   */
  @Override
  public void robotInit() {
    /**
     * Uses the CameraServer class to automatically capture video from USB webcam and send it to the FRC dashboard
     * without doing any vision processing, might change to process image in the future.
     */
    CameraServer.startAutomaticCapture();
    field = new Field2d();

    /**
     * Set up the auto routine selector
     */
    autoChooser.setDefaultOption("do nothing", kNothingAuto);
    autoChooser.addOption("SanAntonio", sanAntonioAuto);
    autoChooser.addOption("cube and mobility", kCubeAuto);
    SmartDashboard.putData("Auto choices", autoChooser);
  }

  /**
   * This method is called every 20 ms, no matter the mode. It runs after
   * the autonomous and teleop specific period methods.
   */
  @Override
  public void robotPeriodic() {
    field.setRobotPose(driveTrain.getPoseMeters());
  }

  @Override
  public void autonomousInit() {

    autoSelected = autoChooser.getSelected();
    System.out.println("Auto selected: " + autoSelected);

    //if (autoSelected == kConeAuto) {
    //  autonomousIntakePower = Constants.INTAKE_OUTPUT_POWER;
    //} else if (autoSelected == kCubeAuto) {
    //  autonomousIntakePower = -Constants.INTAKE_OUTPUT_POWER;
    //}
    autoRoutine = new SanAntonioAuto(driveTrain, armSubsystem, intake);
    autoRoutine.autoInit();
  }

  @Override
  public void autonomousPeriodic() {
    autoRoutine.autoPeriodic();
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {

    if (coDriverPS4.getOptionsButtonPressed()) {
      armSubsystem.toggleArmMode();;
    }

    /**
     * Use the Start button to switch between manual control and set
     */
    if (ArmMode.MANUAL == armSubsystem.getArmMode()) {
      if (coDriverPS4.getL2Button()) {
        /*  lower the arm*/
        armSubsystem.lowerArm();;
      } else if (coDriverPS4.getR2Button()) {
        /*  raise the arm*/
        armSubsystem.raiseArm();
      } else {
        /*  do nothing and let it sit where it is*/
        armSubsystem.neutralArm();
      }
    } else {
      /*  TODO instead of setting the motor, set the PID reference to the armPositionRotation */
      // TODO add button mappings for set points
    }

    if (driverPS4.getL2Button()) {
      intake.intakeCube();
    }
    else if (driverPS4.getR2Button()) {
      intake.ejectCube();
    } else {
      intake.holdCube();
    }

     /*
      * @Hector validate that this logic is working.
      * When the R1 button is first pressed it records the information about where the wheels are loccated.
      * While the R1 button is held down, the robot will try to stay at those targets.
      * Test this by putting the robot up on blocks. Press and hold R1.
      * If the wheels oscillate back and forth, let go of the button and reduce the P gain setting above.
      * If the wheels jitter a little bit, that's ok. They are not under load and it will behave differently on the ground.
      * While holding R1, push on the wheels. If it is squishy and lets you roll it, then increase P.
      * You're looking for the system to allow a small amount of movement and quickly go to pushing harder than you can.
      */

     // This checks the current position when the button is pressed
      if (driverPS4.getR1ButtonPressed()) {
        driveTrain.setBrake();
      }

     if (driverPS4.getR1Button()) {
      /*  breaking*/ 
      driveTrain.brake();
    } else {
      /*  driving */

      // TODO Test the velocity drive, tune kP and kF in the Constants

      // Drive in percent output
      driveTrain.setDriveMotors(-driverPS4.getRawAxis(1), -driverPS4.getRawAxis(2), driverPS4.getL1Button());
      // Drive in velocity
      //driveTrain.setVelocity(intakePower, intakeAmps, armManualControl);(-driverPS4.getRawAxis(1), -driverPS4.getRawAxis(2), driverPS4.getL1Button());
   }
  }
}