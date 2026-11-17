package first.subsystems.drive;

import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.units.measure.Angle;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

public class GyroIOPigeon implements GyroIO {
    
    private final Pigeon2 gyro;

    private final StatusSignal<Angle> yaw, pitch, roll;

    public GyroIOPigeon() {
        this.gyro = new Pigeon2(13, new CANBus(CANPort.CAN_D0));

        this.yaw = this.gyro.getYaw();
        this.pitch = this.gyro.getPitch();
        this.roll = this.gyro.getRoll();

        this.reset();
    }

    @Override
    public void reset() {
        this.gyro.reset();
    }


    @Override
    public Rotation2d getYaw() {
        return new Rotation2d(this.yaw.getValue());
    }

    @Override
    public Rotation2d getPitch() {
        return new Rotation2d(this.pitch.getValue());
    }

    @Override
    public Rotation2d getRoll() {
        return new Rotation2d(this.roll.getValue());
    }

    @Override
    public void refreshAll() {
        BaseStatusSignal.refreshAll(
            this.yaw, 
            this.pitch,
            this.roll
        );
    }


}
