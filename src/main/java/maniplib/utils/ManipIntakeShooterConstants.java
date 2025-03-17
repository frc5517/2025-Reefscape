package maniplib.utils;

import edu.wpi.first.math.system.plant.DCMotor;

public class ManipIntakeShooterConstants {
    public final DCMotor gearbox;
    public final double reduction;
    public final double MOI;

    public ManipIntakeShooterConstants(
            DCMotor gearbox,
            double reduction,
            double MOI
    ) {
        this.gearbox = gearbox;
        this.reduction = reduction;
        this.MOI = MOI;
    }
}
