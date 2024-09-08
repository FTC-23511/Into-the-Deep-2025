package org.firstinspires.ftc.teamcode.drive;

import static com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics.normalizeWheelSpeeds;
import static org.firstinspires.ftc.teamcode.hardware.Globals.LENGTH;
import static org.firstinspires.ftc.teamcode.hardware.Globals.MAXIMUM_MODULE_SPEED;
import static org.firstinspires.ftc.teamcode.hardware.Globals.WIDTH;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.outoftheboxrobotics.photoncore.Photon;

@Photon
public class CoaxialSwerveDrivetrain extends SubsystemBase {
    private final CoaxialSwerveModule fL;
    private final CoaxialSwerveModule fR;
    private final CoaxialSwerveModule bL;
    private final CoaxialSwerveModule bR;

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);

    private SwerveDriveKinematics swerveDriveKinematics;

    // Enter in modules in fL, fR, bL, bR order (f = front, b = back, L = left, R = right)
    // Width and length in meters of c2c of each wheel/pod
    public CoaxialSwerveDrivetrain(CoaxialSwerveModule[] modules) {
        fL = modules[0];
        fR = modules[1];
        bL = modules[2];
        bR = modules[3];

        swerveDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(WIDTH/2, -LENGTH/2),
                new Translation2d(WIDTH/2, LENGTH/2),
                new Translation2d(-WIDTH/2, -LENGTH/2),
                new Translation2d(-WIDTH/2, LENGTH/2)
        );
    }

    public SwerveModuleState[] update(ChassisSpeeds chassisSpeeds) {
        if (chassisSpeeds != null) {
            this.chassisSpeeds = chassisSpeeds;
        }

        // Kinematics math (<3 FTCLib/WPILib) pls work
        SwerveModuleState[] moduleStates = swerveDriveKinematics.toSwerveModuleStates(this.chassisSpeeds);
        normalizeWheelSpeeds(moduleStates, MAXIMUM_MODULE_SPEED);

        // Update individual modules
        fL.update(moduleStates[0].angle.getRadians(), moduleStates[0].speedMetersPerSecond/MAXIMUM_MODULE_SPEED);
        fR.update(moduleStates[1].angle.getRadians(), moduleStates[1].speedMetersPerSecond/MAXIMUM_MODULE_SPEED);
        bL.update(moduleStates[2].angle.getRadians(), moduleStates[2].speedMetersPerSecond/MAXIMUM_MODULE_SPEED);
        bR.update(moduleStates[3].angle.getRadians(), moduleStates[3].speedMetersPerSecond/MAXIMUM_MODULE_SPEED);

        // Only for testing purposes
        return moduleStates;
    }

    public void read() {
        fL.read();
        fR.read();
        bL.read();
        bR.read();
    }

    @Override
    public void periodic() {
        read();
    }
}
