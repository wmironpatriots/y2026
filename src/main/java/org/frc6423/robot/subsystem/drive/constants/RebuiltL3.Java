
package org.frc6423.robot.subsystem.drive.constants;

import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

/**
 * {@link RebuiltL1} but for MK5i /w L3 ratio
 */
public class RebuiltL3 extends RebuiltL1 {
    @Override
    public LinearVelocity getMaxLinearVelocity() {
        // https://www.swervedrivespecialties.com/products/mk5i-swerve-module
        return FeetPerSecond.of(19.9);
    }

    @Override
    public LinearAcceleration getMaxLinearAcceleration() {
        // TODO derive in choreo; values copied from L2 ratio
        return MetersPerSecondPerSecond.of(12.624);
    }

    @Override
    public AngularAcceleration getAngularAcceleration() {
        // TODO derive in choreo; values copied from L2 ratio
        return RadiansPerSecondPerSecond.of(50.022);
    }

    @Override
    public double getDriveRotorToMechRatio() {
        return (54.0 / 16.0) * (25.0 / 32.0) * (30.0 / 15.0);
    }
}
