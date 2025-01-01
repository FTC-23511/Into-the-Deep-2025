package com.seattlesolvers.ftclib.command;

import com.seattlesolvers.ftclib.drivebase.MecanumDrive;
import com.seattlesolvers.ftclib.geometry.Pose2d;
import com.seattlesolvers.ftclib.purepursuit.Path;
import com.seattlesolvers.ftclib.purepursuit.Waypoint;

/**
 * @author Jackson
 * @see Path
 */
public class PurePursuitCommand extends CommandBase {

    private MecanumDrive m_drive;
    private OdometrySubsystem m_odometry;
    private Path m_path;

    public PurePursuitCommand(MecanumDrive drive, OdometrySubsystem odometry, Waypoint... waypoints) {
        m_path = new Path(waypoints);
        m_drive = drive;
        m_odometry = odometry;
    }

    @Override
    public void initialize() {
        m_path.init();
    }

    public void addWaypoint(Waypoint waypoint) {
        m_path.add(waypoint);
    }

    public void addWaypoints(Waypoint... waypoints) {
        for (Waypoint waypoint : waypoints) this.addWaypoint(waypoint);
    }

    public void removeWaypointAtIndex(int index) {
        m_path.remove(index);
    }

    /**
     * Call this in a loop
     */
    @Override
    public void execute() {
        Pose2d robotPose = m_odometry.getPose();
        double[] motorSpeeds = m_path.loop(robotPose.getTranslation().getX(), robotPose.getTranslation().getY(), robotPose.getHeading());
        m_drive.driveRobotCentric(motorSpeeds[0], motorSpeeds[1], motorSpeeds[2]);
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }

    @Override
    public boolean isFinished() {
        return m_path.isFinished();
    }

}
