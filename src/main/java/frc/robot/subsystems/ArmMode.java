package frc.robot.subsystems;

public enum ArmMode {
    MANUAL("Manual"),
    POSITION("Position");

    public final String label;

    private ArmMode(String label) {
        this.label = label;
    }
}
