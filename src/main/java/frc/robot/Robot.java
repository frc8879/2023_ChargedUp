// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
//import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.AutoRoutine;
import frc.robot.autonomous.ChargingStationMobilityBalance;
import frc.robot.autonomous.SideAuto;
import frc.robot.autonomous.SideAutoRotate;
import frc.robot.subsystems.ArmMode;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;

public class Robot extends TimedRobot {
  /*
   * Autonomous selection options.
   */
  private static final String chargingStationAuto = "Charging Station auto";
  private static final String sideAuto = "Side Auto";
  private static final String sideAutoRotate = "Side Auto Rotate";
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
  XboxController driver = new XboxController(0);
  XboxController coDriver = new XboxController(1);

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
    autoChooser.addOption("Charging Station auto", chargingStationAuto);
    autoChooser.addOption("Side Auto", sideAuto);
    autoChooser.addOption("Side Auto Rotate", sideAutoRotate);
    SmartDashboard.putData("Auto choices", autoChooser);
  }

  /**
   * This method is called every 20 ms, no matter the mode. It runs after
   * the autonomous and teleop specific period methods.
   */
  @Override
  public void robotPeriodic() {
    field.setRobotPose(driveTrain.getPoseMeters());
    SmartDashboard.putNumber("Arm Position:", armSubsystem.getPosition());

    armSubsystem.periodic();
    driveTrain.periodic();
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
    switch (autoSelected) {
      case chargingStationAuto: {
        autoRoutine = new ChargingStationMobilityBalance(driveTrain, armSubsystem, intake);
      } break;
      case sideAuto: {
        autoRoutine = new SideAuto(driveTrain, armSubsystem, intake);
      } break;
      case sideAutoRotate: {
        System.out.println("Side Auto Rotate");
        autoRoutine = new SideAutoRotate(driveTrain, armSubsystem, intake);
      }
      //default: {
        //System.out.println("defaulting");
        //autoRoutine = new SideAuto(driveTrain, armSubsystem, intake);
      //}
    }
    
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

    if (coDriver.getStartButtonPressed()) {
      armSubsystem.toggleArmMode();
    }

    /**
     * Use the Start button to switch between manual control and set
     */
    //if (ArmMode.MANUAL == armSubsystem.getArmMode()) {
      //if (coDriver.getLeftTriggerAxis() > 0.5) {
      //  /*  lower the arm*/
      //  armSubsystem.lowerArm();
      //} else if (coDriver.getRightTriggerAxis() > 0.5) {
      //  /*  raise the arm*/
      //  armSubsystem.raiseArm();
      //} else {
      //  /*  do nothing and let it sit where it is*/
      //  armSubsystem.neutralArm();
      //}
    //} else {
    //}
      ///*  TODO instead of setting the motor, set the PID reference to the armPositionRotation */

      // Button mappings for target arm set points
      if (coDriver.getAButtonPressed()) {
        armSubsystem.setPosition(-0.32);
      }
      if (coDriver.getBButtonPressed()) {
        armSubsystem.setPosition(-0.27);
      }
      if (coDriver.getXButtonPressed()) {
        armSubsystem.setPosition(-0.111);
      }
      if(coDriver.getYButtonPressed()){
        armSubsystem.setPosition(Constants.ARM_POSITION_STOWED);
      }
    if (intake.getAmps() > (50)){
      armSubsystem.setPosition(Constants.ARM_POSITION_STOWED);

    }
    else if (driver.getRightTriggerAxis() > 0.5) {
      intake.intakeCube();
    }
    else if (driver.getLeftTriggerAxis() > 0.5) {
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
      if (driver.getRightBumperPressed()) {
        driveTrain.setBrake();
      }

     if (driver.getRightBumper()) {
      /*  breaking*/ 
      driveTrain.brake();
    } else {
      /*  driving */

      // TODO Test the velocity drive, tune kP and kF in the Constants

      // Drive in percent output
      //driveTrain.setDriveMotors(-driver.getRawAxis(1), -driver.getRawAxis(4), driver.getLeftBumper());
      // Drive in velocity
      driveTrain.setVelocity(-driver.getRawAxis(1), -driver.getRawAxis(4), driver.getLeftBumper());
   }
  }
}
