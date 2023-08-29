package frc.robot.subsystems;

import frc.robot.Constants;

import java.lang.annotation.Target;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    CANSparkMax arm;
    ProfiledPIDController armPIDController;
    RelativeEncoder armEncoder;
    boolean armManualControl = false;
    private ArmMode armMode;
    private SparkMaxPIDController maxPIDController;

    public ArmSubsystem() {
        arm = new CANSparkMax(Constants.ARM_ID, MotorType.kBrushless);

        arm.setInverted(true);
        arm.setIdleMode(IdleMode.kBrake);
        arm.setSmartCurrentLimit(Constants.ARM_CURRENT_LIMIT_A);
        armEncoder = arm.getEncoder();
        armEncoder.setPosition(0.0);
        armEncoder.setPositionConversionFactor(1.0 / 75.0); // TODO Double check the gear ratio on the motor
        //armPIDController = new ProfiledPIDController(2, 0, 0, new TrapezoidProfile.Constraints(4, 4));
        //armPIDController.setTolerance(0.02);
        maxPIDController = arm.getPIDController();
        maxPIDController.setOutputRange(-1.0, 1.0);
        maxPIDController.setP(2.0);
        maxPIDController.setFF(0.1);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Position:", armEncoder.getPosition());
        //if (armMode == ArmMode.POSITION) {
            SmartDashboard.putNumber("Arm Target:", target);
            //armPIDController.setGoal(target);
        maxPIDController.setReference(target, CANSparkMax.ControlType.kPosition);
        //}   
    }

    /**
     * Set the arm output power. Positive is out, negative is in.
     * 
     * @param percent
     */
    private void setArmMotor(double percent) {
        arm.set(percent);
        SmartDashboard.putNumber("arm power (%)", percent);
        SmartDashboard.putNumber("arm motor current (amps)", arm.getOutputCurrent());
        SmartDashboard.putNumber("arm motor temperature (C)", arm.getMotorTemperature());
    }

    //public void raiseArm() {
    //    this.setArmMotor(Constants.ARM_OUTPUT_POWER);
    //}

    //public void lowerArm() {
    //    this.setArmMotor(-Constants.ARM_OUTPUT_POWER);
    //}

    public void neutralArm() {
        this.setArmMotor(0.0);
    }

    //public void setArmMode(ArmMode armMode) {
    //    this.armMode = armMode;

    //    SmartDashboard.putString("Arm Control Mode", getArmMode().label);
    //}

    public void toggleArmMode() {
        if (ArmMode.MANUAL == this.armMode) {
            this.armMode = ArmMode.POSITION;
        } else {
            this.armMode = ArmMode.MANUAL;
        }

        SmartDashboard.putString("Arm Control Mode", getArmMode().label);
    }

    public ArmMode getArmMode() {
        return this.armMode;
    }

    private double computeArmOutput() {
        double output = armPIDController.calculate(armThetaFromNEOEncoder()) + computeArmArbitraryFeedForward();
        SmartDashboard.putNumber("arm output", output);
        return output;
    }

    private double armThetaFromNEOEncoder() {
        double rawPos = armEncoder.getPosition();
        double conversion = 1.0; // The encoder gearing is set, so we probably don't need a scalar here
        double offset = 0.0; // This is the offset from the starting position
        SmartDashboard.putNumber("raw NEO Encoder", rawPos);
        double theta = Math.toRadians(rawPos * (conversion) + offset);
        SmartDashboard.putNumber("arm theta", theta);
        return theta;
    }

    private final double STALLED_TORQUE = 1.3; // This is half the stall torque for the 6377 shoulder which has similar
                                               // gearing but 2 motors instead of one.
    private double target;

    private double computeArmArbitraryFeedForward() {
        double theta = armThetaFromNEOEncoder();
        double centerOfMass = 0.5; // TODO get correct distance to center of mass
        double mass = 4.03; // TODO get correct mass
        double gearRatio = 90; // TODO get correct gearing
        double numMotors = 1;
        double torque = Math.cos(theta) * 9.81 * mass * centerOfMass;
        double out = torque / (STALLED_TORQUE * 0.85 * gearRatio * numMotors);
        SmartDashboard.putNumber("arm arb ffw", out);
        return out;
    }

    public double getPosition() {
        return armEncoder.getPosition();
    }

    public void setPosition(double targetPosition) {
        target = targetPosition;
    }
}
