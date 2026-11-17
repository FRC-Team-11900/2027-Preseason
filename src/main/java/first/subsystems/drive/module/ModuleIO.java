package first.subsystems.drive.module;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;

public interface ModuleIO {

    public Distance getDrivePosition();

    public LinearVelocity getDriveVelocity();
    
    public Rotation2d getSteerPosition();

    public AngularVelocity getSteerVelocity();

    public SwerveModulePosition getPosition();

    public SwerveModuleVelocity getVelocity();

    public void setVelocities(SwerveModuleVelocity velocities);

    public void refreshAll();

}
