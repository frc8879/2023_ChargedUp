package frc.robot.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmMode;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;

public class SideAutoRotate implements AutoRoutine {

    private final DriveTrain driveTrain;
    private final ArmSubsystem arm;
    private final Intake intake;
    private final Timer autoTime;
    private double pauseStart;

    private enum Action {
    
        SHOOT,
        RETRACT,
        MOBILITY,
        ROTATE,
        BRAKE
    }

    private Action action;
    private double backwardsHeading;
    private double sumcheck;
    private double subcheck;

    public SideAutoRotate(DriveTrain driveTrain, ArmSubsystem arm, Intake intake) {
        this.driveTrain = driveTrain;
        this.arm = arm;
        this.intake = intake;
        this.autoTime = new Timer();
    }

    @Override
    public void autoInit() {
        autoTime.reset();
        autoTime.start();     
        driveTrain.resetPosition();
        this.action = Action.SHOOT;
    }

    @Override
    public void autoPeriodic() {
        arm.periodic();
        switch(action){
            case SHOOT: {
                intake.ejectCube();
                if(autoTime.get() > 0.6) {
                    action = Action.RETRACT;
                    intake.idle();
                }
            } break;
            case RETRACT: {
                arm.setPosition(Constants.ARM_POSITION_STOWED);
                if(autoTime.get() > 0.8) {
                    action = Action.MOBILITY;
                }
            } break;
            case MOBILITY: {
                System.out.println("Mobility: ");

                // drive backwards over the charging station
                driveTrain.setVelocity(-0.8, 0.0, false);
                // Distnace to leave community is 16' 1 1/4"
                // Gearing is 8.45 NEO revolutions for 1 wheel revolution - 1 wheel revolution is 18.85"
                // This comes out to 86.63 revolutions
                if (driveTrain.getDistance() < -4) {
                    pauseStart = autoTime.get();
                    backwardsHeading = driveTrain.getHeading();
                    action = Action.ROTATE;
                }
            } break;
            case ROTATE: {
                System.out.println("Rotating: " + backwardsHeading + "\t" + driveTrain.getHeading());
                sumcheck = 180 + backwardsHeading;
                subcheck = backwardsHeading - 180;
                // rotate the robot until it is at 180 from the target
                driveTrain.setDriveMotors(0.0, -0.4, false);
                if(driveTrain.getHeading()>= sumcheck || driveTrain.getHeading()<=subcheck){
                    //driveTrain.setDriveMotors(0.0, 0.0, false);
                    driveTrain.setBrake();
                    action = Action.BRAKE;
                }
                
            }break;
            case BRAKE: {
                driveTrain.brake();
                arm.setPosition(-0.32);
            }

        }

    }
    
}
