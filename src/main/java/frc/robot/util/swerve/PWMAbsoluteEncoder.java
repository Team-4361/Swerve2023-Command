package frc.robot.util.swerve;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import swervelib.encoders.SwerveAbsoluteEncoder;

import static edu.wpi.first.math.util.Units.radiansToDegrees;

public class PWMAbsoluteEncoder extends SwerveAbsoluteEncoder {

    private final DutyCycleEncoder encoder;
    private boolean inverted = false;

    public PWMAbsoluteEncoder(int id) {
        this.encoder = new DutyCycleEncoder(id);
    }

    /**
     * Reset the encoder to factory defaults.
     */
    @Override
    public void factoryDefault() {

    }

    /**
     * Clear sticky faults on the encoder.
     */
    @Override
    public void clearStickyFaults() {

    }

    /**
     * Configure the absolute encoder to read from [0, 360) per second.
     *
     * @param inverted Whether the encoder is inverted.
     */
    @Override
    public void configure(boolean inverted) {
        this.inverted = inverted;
    }

    /**
     * Get the absolute position of the encoder.
     *
     * @return Absolute position in degrees from [0, 360).
     */
    @Override
    public double getAbsolutePosition() {
        if (inverted) {
            return radiansToDegrees(-encoder.get() * 2 * Math.PI)%360;
        } else {
            return radiansToDegrees(encoder.get() * 2 * Math.PI)%360;
        }
    }

    /**
     * Get the instantiated absolute encoder Object.
     *
     * @return Absolute encoder object.
     */
    @Override
    public Object getAbsoluteEncoder() {
        return encoder;
    }
}
