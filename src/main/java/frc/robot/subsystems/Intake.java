package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{

    CANSparkMax intake;

    public Intake() {
        intake = new CANSparkMax(Constants.INTAKE_ID, MotorType.kBrushless);
        intake.setInverted(false);
        intake.setIdleMode(IdleMode.kBrake);
    }
    
      /**
     * Set the intake output power.
     * 
     * @param percent desired speed
     * @param amps current limit
     */
    private void setIntakeMotor(double percent, int amps) {
        intake.set(percent);
        intake.setSmartCurrentLimit(amps);

        SmartDashboard.putNumber("intake power (%)", percent);
        SmartDashboard.putNumber("intake motor current (amps)", intake.getOutputCurrent());
        SmartDashboard.putNumber("intake motor temperature (C)", intake.getMotorTemperature());
    }

    public void intakeCube() {
        setIntakeMotor(Constants.INTAKE_OUTPUT_POWER, Constants.INTAKE_CURRENT_LIMIT_A);
    }

    public void ejectCube() {
        setIntakeMotor(-Constants.INTAKE_OUTPUT_POWER, Constants.INTAKE_CURRENT_LIMIT_A);
    }

    public void holdCube() {
        setIntakeMotor(Constants.INTAKE_HOLD_POWER, Constants.INTAKE_HOLD_CURRENT_LIMIT_A);
    }

    public void idle() {
        setIntakeMotor(0.0, 40);
    }
}
