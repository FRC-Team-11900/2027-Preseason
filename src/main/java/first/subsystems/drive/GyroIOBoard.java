package first.subsystems.drive;


import org.wpilib.hardware.imu.OnboardIMU;
import org.wpilib.hardware.imu.OnboardIMU.MountOrientation;
import org.wpilib.math.geometry.Rotation2d;

public class GyroIOBoard implements GyroIO {
    
    private final OnboardIMU gyro;

    public GyroIOBoard() {
        this.gyro = new OnboardIMU(MountOrientation.FLAT);

        this.reset();
    }

    @Override
    public void reset() {
        this.gyro.resetYaw();
    }

    @Override
    public Rotation2d getYaw() {
        return this.gyro.getRotation2d();
    }

    @Override
    public Rotation2d getPitch() {
        return new Rotation2d(this.gyro.getAngleY());
    }

    @Override
    public Rotation2d getRoll() {
        return new Rotation2d(this.gyro.getAngleX());
    }
}
