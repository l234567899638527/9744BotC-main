package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(21));

    public static final int Mod1 = 1;
    public static final int Mod2 = 2;
    public static final int Mod3 = 3;
    public static final int Mod4 = 4;
    public static final int Mod5 = 5;
    public static final int Mod6 = 6;
    public static final int Mod7 = 7;
    public static final int Mod8 = 8;


    public static final float maxSpeed = 4.5f;

    public static final float normalDspeed = 0.6f;
    public static final float rotationSpeed = 0.45f; 
}