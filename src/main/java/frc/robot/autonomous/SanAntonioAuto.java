package frc.robot.autonomous;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;

public class SanAntonioAuto implements AutoRoutine {

    private final DriveTrain driveTrain;
    private final ArmSubsystem arm;
    private final Intake intake;
    private final Timer autoTime;

    public SanAntonioAuto(DriveTrain driveTrain, ArmSubsystem arm, Intake intake) {
        this.driveTrain = driveTrain;
        this.arm = arm;
        this.intake = intake;
        this.autoTime = new Timer();
    }

    @Override
    public void autoInit() {
        autoTime.reset();
        autoTime.start();        
    }

    @Override
    public void autoPeriodic() {
        if(autoTime.get() < 2.0){
            //Shoot 
            intake.ejectCube();
      
          } else if(autoTime.get() < 8.0){
            //Go Backwards
            driveTrain.setDriveMotors(-0.5, 0.0, false);
            intake.idle();;
          } else {
            driveTrain.setDriveMotors(0, 0, false);
          }
    }
    
}