package first.subsystems.drive;

import org.wpilib.math.geometry.Rotation2d;

public interface GyroIO {

    public void reset();

    public Rotation2d getYaw();

    public Rotation2d getPitch();

    public Rotation2d getRoll();

    public void refreshAll();
}
