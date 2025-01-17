package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.config.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Autonomous(name = "AutoLeftSideBackup_3pcs", group = "Examples")
public class AutoTest3BackUpWorking01102025 extends OpMode {

    private Follower follower;
    private Timer opmodeTimer;
    private int pathState = 0;
    private int loopNumber = 0;

    private double positionTol = 1;
    // position tolerance for the robot


    // private int caseNumber = 0;
    // Starting and target positions
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose scorePose = new Pose(6.36, 17., 2.37);
    private final Pose pickup1Pose = new Pose(15.8, 9.5, 3.15);
    private final Pose pickup2Pose = new Pose(15.8, 17.98, Math.toRadians(180));
    private final Pose pickup3Pose = new Pose(35.6, 9.84, 4.7);
    private final Pose parkPose = new Pose(-51, 10.5, Math.toRadians(270));
    private final Pose parkControlPose = new Pose(77, 24, Math.toRadians(270));

    // Paths and subsystems
    private Path scorePreload, park;
    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;
    private ClawSubsystem clawSubsystem;

    // Initialization flag
    // private boolean initialized = false;
    private boolean sampleInBasket = true;
    private boolean sampleInGraber = false;


    @Override
    public void init() {
        follower = new Follower(hardwareMap);
        clawSubsystem = new ClawSubsystem(hardwareMap);

        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower.setStartingPose(startPose);
        clawSubsystem.moveDownSp();
        buildPaths();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        clawSubsystem.moveDownSp();
        if (clawSubsystem.height != 0) {
            clawSubsystem.moveDownSp();
        }

    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        loopNumber++;
        telemetry.addData("Path State", pathState);
        telemetry.addData("loop number", loopNumber);
        // telemetry.addData("Case number", caseNumber);

        //telemetry.addData("X", follower.getPose().getX());
        //telemetry.addData("Y", follower.getPose().getY());
        //telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Elapsed Time", opmodeTimer.getElapsedTimeSeconds());

        telemetry.addData("Elevator Height", clawSubsystem.height);
        telemetry.addData("Elevator Current Position", clawSubsystem.getElevatorPosition());
        telemetry.addData("Robot is at pick up position Pickup 1", isNearPose(follower.getPose(), pickup1Pose, positionTol));
        telemetry.addData("Robot is at pick up position Pickup 2", isNearPose(follower.getPose(), pickup2Pose, positionTol));
        telemetry.addData("Robot is at pick up position Pickup 3", isNearPose(follower.getPose(), pickup3Pose, positionTol));

        telemetry.addData("Robot is at Score position", isNearPose(follower.getPose(), scorePose, 0.5));

        telemetry.addData("Sample in buket position", sampleInBasket);
        telemetry.addData("Sample in grabber position", sampleInGraber);

        telemetry.addData("elevator Height", clawSubsystem.height);
        telemetry.addData("elevator Current Positon", clawSubsystem.getElevatorPosition());


        telemetry.update();
    }

    @Override
    public void stop() {
        // Stop all systems if necessary
    }

    private void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        park = new Path(new BezierCurve(new Point(scorePose), new Point(parkControlPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload, true);

                if(clawSubsystem.height != 2) {
                    clawSubsystem.moveUp();
                }
                else if (clawSubsystem.height == 2) {
                    opmodeTimer.resetTimer();
                    setPathState(1);
                    //telemetry.addData("PathState", "1");
                }
                break;
            case 1:
                // Dump bucket object and move to next position
                if (isNearPose(follower.getPose(), scorePose, 0.5 )){
                    if (sampleInBasket) {
                        if (clawSubsystem.height != 2) {
                            clawSubsystem.moveUp();
                        }
                        else {
                            // elevator is at high position, release bucket object
                            clawSubsystem.bucket();
                            sampleInBasket = false;
                            opmodeTimer.resetTimer();
                            setPathState(2);
                        }
                    }
                }
                break;
            case 2:
// Lower elevator, follow path, and release claw object
                if(clawSubsystem.height != 0 && opmodeTimer.getElapsedTimeSeconds() > 1) {
                    clawSubsystem.grabVertical();
                    clawSubsystem.moveDown();
                    follower.followPath(grabPickup1, true);
                    opmodeTimer.resetTimer();
                    setPathState(3);
                }
                break;
            case 3:
                // picking up the first sample
                if (isNearPose(follower.getPose(), pickup1Pose, 1)){
                    if (clawSubsystem.height != 0) {
                        clawSubsystem.moveDown();
                        clawSubsystem.grabVertical();
                    }
                    if (clawSubsystem.height == 0) {
                        if (!sampleInBasket) {
                            if (!sampleInGraber) {
                                clawSubsystem.grab();
                                if (opmodeTimer.getElapsedTimeSeconds() > 3.5) {
                                    clawSubsystem.release();
                                    sampleInBasket=true;
                                    setPathState(4);
                                }
                            }
                        }
                    }
                }
                break;

            case 4:
                // move first sample to dumping place
                follower.followPath(scorePickup1, true);
                if(clawSubsystem.height != 2) {
                    if (opmodeTimer.getElapsedTimeSeconds() > 1.5) {
                        clawSubsystem.moveUp();
                    }
                }
                else if (clawSubsystem.height == 2) {
                    opmodeTimer.resetTimer();
                    setPathState(5);
                }
                break;
            case 5:
                // Dump bucket object and move to next position, like case 1
                if (isNearPose(follower.getPose(), scorePose, 1)){
                    if (sampleInBasket) {
                        if (clawSubsystem.height != 2) {
                            clawSubsystem.moveUp();
                        }
                        else {
                            // elevator is at high position, release bucket object
                            clawSubsystem.bucket();
                            sampleInBasket = false;
                            opmodeTimer.resetTimer();
                            setPathState(6);
                        }
                    }
                }
                break;

            case 6:
                if(clawSubsystem.height != 0 && opmodeTimer.getElapsedTimeSeconds() > 1) {
                    // note, change the time to 3 to wait longer time to move.
                    clawSubsystem.grabVertical();
                    clawSubsystem.moveDown();
                    follower.followPath(grabPickup2, true);
                    opmodeTimer.resetTimer();
                    setPathState(7);
                }
                break;
            case 7:
                if (isNearPose(follower.getPose(), pickup2Pose, positionTol)) {
                    if (clawSubsystem.height != 0) {
                        clawSubsystem.moveDown();
                        clawSubsystem.grabVertical();
                    }
                    if (clawSubsystem.height == 0) {
                        if (!sampleInBasket) {
                            if (!sampleInGraber) {
                                clawSubsystem.grab();
                                if (opmodeTimer.getElapsedTimeSeconds() >3.5) {
                                    clawSubsystem.release();
                                    sampleInBasket = true;
                                    setPathState(8);
                                }
                            }
                        }
                    }
                }
                break;

            case 8:
                follower.followPath(scorePickup2, true);
                if(clawSubsystem.height != 2) {
                    if (opmodeTimer.getElapsedTimeSeconds() > 2) {
                        clawSubsystem.moveUp();
                    }
                }
                else {
                    opmodeTimer.resetTimer();
                    setPathState(9);
                    //telemetry.addData("PathState", "1");
                }
                break;

            case 9:
                // Dump bucket object and move to next position, like case 1
                if (isNearPose(follower.getPose(), scorePose, 1)){
                    if (sampleInBasket) {
                        if (clawSubsystem.height != 2) {
                            clawSubsystem.moveUp();
                        }
                        else {
                            // elevator is at high position, release bucket object
                            clawSubsystem.bucket();
                            sampleInBasket = false;
                            opmodeTimer.resetTimer();
                            setPathState(10);
                        }
                    }
                }
                break;

            case 10:

                if(clawSubsystem.height != 0 && opmodeTimer.getElapsedTimeSeconds() > 1) {
                    // note, change the time to 3 to wait longer time to move.
                    clawSubsystem.grabVertical();
                    clawSubsystem.moveDown();
                    follower.followPath(grabPickup3, true);
                    opmodeTimer.resetTimer();
                    setPathState(-1);
                }
                break;

                // set to stop at pose 3 to save time
            case 11:

                //follower.followPath(grabPickup3, true);
                if (isNearPose(follower.getPose(), pickup3Pose, positionTol)) {
                    if (clawSubsystem.height != 0) {
                        clawSubsystem.moveDown();
                        clawSubsystem.grabVertical();
                    }
                    if (clawSubsystem.height == 0) {
                        if (!sampleInBasket) {
                            if (!sampleInGraber) {
                                clawSubsystem.grab();
                                if (opmodeTimer.getElapsedTimeSeconds() >3.5) {
                                    clawSubsystem.release();
                                    sampleInBasket = true;
                                    setPathState(12);
                                }
                            }
                        }
                    }
                }
                break;

            case 12:
                follower.followPath(scorePickup3, true);
                if(clawSubsystem.height != 2) {
                    if (opmodeTimer.getElapsedTimeSeconds() > 2) {
                        clawSubsystem.moveUp();
                    }
                }
                else {
                    opmodeTimer.resetTimer();
                    setPathState(13);
                    //telemetry.addData("PathState", "1");
                }
                break;


            case 13:

                if (isNearPose(follower.getPose(), scorePose, positionTol)){
                    if (sampleInBasket) {
                        if (clawSubsystem.height != 2) {
                            clawSubsystem.moveUp();
                        }
                        else {
                            // elevator is at high position, release bucket object
                            clawSubsystem.bucket();
                            sampleInBasket = false;
                            opmodeTimer.resetTimer();
                            setPathState(14);
                        }
                    }
                }
                break;

            case 14:

                if(clawSubsystem.height != 0 && opmodeTimer.getElapsedTimeSeconds() > 1) {
                    // note, change the time to 3 to wait longer time to move.
                    clawSubsystem.grabVertical();
                    clawSubsystem.moveDown();
                    follower.followPath(park, true);
                    opmodeTimer.resetTimer();
                    setPathState(15);
                }
                break;

            case 15:
                // follower.followPath(park, true);
                if (isNearPose(follower.getPose(), parkPose, positionTol)) {
                    if (clawSubsystem.height != 0) {
                        clawSubsystem.moveDown();
                        clawSubsystem.grabVertical();
                    }
                    setPathState(-1);
                }
                break;

            case -1:
                telemetry.addData("Status", "Autonomous Complete");
                break;
        }
    }

    private void setPathState(int state) {
        pathState = state;
        opmodeTimer.resetTimer();
    }
    private boolean isNearPose(Pose current, Pose target, double tolerance) {
        return (Math.abs(current.getX() - target.getX()) < tolerance &&
                Math.abs(current.getY() - target.getY()) < tolerance) ;
    }


}
