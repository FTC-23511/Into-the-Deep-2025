package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.seattlesolvers.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;

// Thanks Powercube/Watt-sUP 16166 we copied verbatim
public class FollowPathCommand extends CommandBase {

    private final Follower follower;
    private final PathChain path;
    private boolean holdEnd = true;

    public FollowPathCommand(Follower follower, PathChain path) {
        this.follower = follower;
        this.path = path;
    }

    public FollowPathCommand(Follower follower, Path path) {
        this(follower, new PathChain(path));
    }

    /**
     * Decides whether or not to make the robot maintain its position once the path ends.
     *
     * @param holdEnd If the robot should maintain its ending position
     * @return This command for compatibility in command groups
     */
    public FollowPathCommand setHoldEnd(boolean holdEnd) {
        this.holdEnd = holdEnd;
        return this;
    }

    @Override
    public void initialize() {
        follower.followPath(path, holdEnd);
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy();
    }
}