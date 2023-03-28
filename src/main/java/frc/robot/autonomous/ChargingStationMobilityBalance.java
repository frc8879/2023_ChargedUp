package frc.robot.autonomous;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.ArmMode;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;

public class ChargingStationMobilityBalance implements AutoRoutine {

    private final DriveTrain driveTrain;
    private final ArmSubsystem arm;
    private final Intake intake;
    private final Timer autoTime;
    private double pauseStart;

    private enum Action {
        SHOOT,
        RETRACT,
        MOBILITY,
        PAUSE,
        CHARGING_STATION,
        BALANCE,
        BRAKE
    }

    private Action action;

    public ChargingStationMobilityBalance(DriveTrain driveTrain, ArmSubsystem arm, Intake intake) {
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
        switch(action) {
            case SHOOT: {
                intake.ejectCube();
                if(autoTime.get() > 0.6) {
                    action = Action.RETRACT;
                    intake.idle();
                }
            } break;
            case RETRACT: {
                arm.setPosition(Constants.ARM_POSITION_STOWED);
                action = Action.MOBILITY;
            } break;
            case MOBILITY: {
                // drive backwards over the charging station
                driveTrain.setVelocity(-0.4, 0.0, false);
                // Distnace to leave community is 16' 1 1/4"
                // Gearing is 8.45 NEO revolutions for 1 wheel revolution - 1 wheel revolution is 18.85"
                // This comes out to 86.63 revolutions
                if (driveTrain.getDistance() >= 86.63) {
                    driveTrain.setBrake();
                    pauseStart = autoTime.get();
                    action = Action.PAUSE;
                }
            } break;
            case PAUSE: {
                driveTrain.brake();
                if ( (autoTime.get() - pauseStart) > 0.4 ) {
                    action = Action.CHARGING_STATION;
                }
            } break;
            case CHARGING_STATION: {
                // drive forward onto the charging station
                driveTrain.setVelocity(0.4, 0.0, false);
                // Chargin station is 6' 4 1/8" - Target move is 3' 2"
                // Approximate revolutions is 17.03
                if (driveTrain.getDistance() < 69.6) {
                    action = Action.BALANCE;
                }
            } break;
            case BALANCE: {
                // Test some tuning on the distances first, then we can look at using the pigeon to balance.
                driveTrain.setBrake();
            } break;
            case BRAKE: {
                driveTrain.brake();
            }

        }

    }
    
}