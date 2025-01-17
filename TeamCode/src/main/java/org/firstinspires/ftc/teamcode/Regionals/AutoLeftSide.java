package org.firstinspires.ftc.teamcode.Regionals;

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

@Autonomous(name = "NotUsed", group = "Examples")
public class AutoLeftSide extends OpMode {

    private Follower follower;
    private Timer opmodeTimer;
    private int pathState = 0;
    private int loopNumber = 0;
    private int caseNumber = 0;
    // Starting and target positions
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose scorePose = new Pose(6, 17.8, Math.toRadians(45));
    private final Pose pickup1Pose = new Pose(15.5, 7, 3.14);
    private final Pose pickup2Pose = new Pose(12.5, 17, Math.toRadians(180));
    private final Pose pickup3Pose = new Pose(36, 8.1, 4.64);
    private final Pose parkPose = new Pose(-55, 10, Math.toRadians(270));
    private final Pose parkControlPose = new Pose(77, 24, Math.toRadians(270));

    // Paths and subsystems
    private Path scorePreload, park;
    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;
    private ClawSubsystem clawSubsystem;

    // Initialization flag
    private boolean initialized = false;
    private boolean sampleInBasket = true;

    @Override
    public void init() {
        follower = new Follower(hardwareMap);
        clawSubsystem = new ClawSubsystem(hardwareMap);

        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        loopNumber++;
        telemetry.addData("Path State", pathState);
        telemetry.addData("loop number", loopNumber);
        telemetry.addData("Case number", caseNumber);

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Elapsed Time", opmodeTimer.getElapsedTimeSeconds());
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
                if (!initialized) {
                    clawSubsystem.moveUp(); // Move elevator up
                    initialized = true;
                }
                if (opmodeTimer.getElapsedTimeSeconds() > 0) {
                    // Reduced by 2 seconds
                    follower.followPath(scorePreload, true);
                    opmodeTimer.resetTimer();
                    setPathState(1);

                    //telemetry.addData("PathState", "1");
                }
                break;

            case 1:
                if (follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1)) {
                    if (opmodeTimer.getElapsedTimeSeconds() > 3) {
                        if (sampleInBasket) {
                            caseNumber++;
                            clawSubsystem.bucket();
                            clawSubsystem.reset();// Reduced by 2 seconds
                            opmodeTimer.resetTimer();
                            sampleInBasket = false;
                        }
                    }


                    if (opmodeTimer.getElapsedTimeSeconds() > 1) {
                        if (!sampleInBasket){// Reduced by 2 seconds
                            clawSubsystem.moveDown();
                            setPathState(2);// Release object
                        }
                    }
                }




                break;

            case 2:
                if (opmodeTimer.getElapsedTimeSeconds() > 3) {

                    clawSubsystem.release();// Reduced by 2 seconds
                    follower.followPath(grabPickup1, true);
                    opmodeTimer.resetTimer();
                    setPathState(3);
                    opmodeTimer.resetTimer();
                }
                break;

            case 3:
                if (follower.getPose().getX() > (pickup1Pose.getX() - 1) && follower.getPose().getY() > (pickup1Pose.getY() - 1)) {
                    if (opmodeTimer.getElapsedTimeSeconds() > 4) {  // Reduced by 2 seconds
                        clawSubsystem.release();
                        sampleInBasket = true;
                        opmodeTimer.resetTimer();
                        initialized = false;
                        setPathState(4);

                    }
                    else if (opmodeTimer.getElapsedTimeSeconds() < 3) {
                        if (opmodeTimer.getElapsedTimeSeconds() > 2) {
                            clawSubsystem.grab(); // Grab object
                        }
                    }
                }
                break;

            case 4:
                if (!initialized) {
                    if (opmodeTimer.getElapsedTimeSeconds() > 1) {
                        clawSubsystem.moveUp(); // Move elevator up
                        initialized = true;
                    }}
                if (opmodeTimer.getElapsedTimeSeconds() > 3.7) {
                    // Reduced by 2 seconds
                    follower.followPath(scorePickup2, true);
                    opmodeTimer.resetTimer();
                    setPathState(-1);

                    //telemetry.addData("PathState", "1");
                }
                break;

            case 5:

                if (follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1)) {
                    if (opmodeTimer.getElapsedTimeSeconds() > 3) {
                        if (sampleInBasket) {
                            caseNumber++;
                            clawSubsystem.bucket();
                            clawSubsystem.reset();// Reduced by 2 seconds
                            opmodeTimer.resetTimer();
                            sampleInBasket = false;
                        }
                    }


                    if (opmodeTimer.getElapsedTimeSeconds() > 3.1) {
                        if (!sampleInBasket){// Reduced by 2 seconds
                            clawSubsystem.moveDown(); //
                            setPathState(-1);// Release object
                        }
                    }
                }

            case 6:
                if (opmodeTimer.getElapsedTimeSeconds() > 3) {  // Reduced by 2 seconds
                    follower.followPath(grabPickup2);
                    setPathState(-1);
                }
                break;

            case 7:
                if (isCloseTo(pickup2Pose)) {
                    clawSubsystem.grab(); // Grab object
                    opmodeTimer.resetTimer();
                    if (opmodeTimer.getElapsedTimeSeconds() > 1) {  // Reduced by 2 seconds
                        clawSubsystem.release();
                        opmodeTimer.resetTimer();
                        setPathState(8);
                        initialized = false;
                    }
                }
                break;

            case 8:
                if (!initialized) {
                    clawSubsystem.moveUp();  // Move elevator up
                    initialized = true;
                }
                if (opmodeTimer.getElapsedTimeSeconds() > 3) {  // Reduced by 2 seconds
                    follower.followPath(scorePickup2);
                    setPathState(9);
                }
                break;

            case 9:
                if (isCloseTo(scorePose)) {
                    clawSubsystem.bucket();
                    clawSubsystem.grab();
                    opmodeTimer.resetTimer();
                    if (opmodeTimer.getElapsedTimeSeconds() > 1) {  // Reduced by 2 seconds
                        clawSubsystem.moveDown(); // Release object
                        opmodeTimer.resetTimer();
                        setPathState(2);
                    }
                    if (opmodeTimer.getElapsedTimeSeconds() > 3) {  // Reduced by 2 seconds
                        clawSubsystem.release();
                    }
                }
                break;

            case 10:
                if (opmodeTimer.getElapsedTimeSeconds() > 3) {  // Reduced by 2 seconds
                    follower.followPath(grabPickup3);
                    setPathState(11);
                }
                break;

            case 11:
                if (isCloseTo(pickup3Pose)) {
                    clawSubsystem.grab(); // Grab object
                    opmodeTimer.resetTimer();
                    if (opmodeTimer.getElapsedTimeSeconds() > 1) {  // Reduced by 2 seconds
                        clawSubsystem.release();
                        opmodeTimer.resetTimer();
                        setPathState(12);
                    }
                }
                break;

            case 12:
                if (!initialized) {
                    clawSubsystem.moveUp();  // Move elevator up
                    initialized = true;
                }
                if (opmodeTimer.getElapsedTimeSeconds() > 4) {  // Reduced by 2 seconds
                    follower.followPath(scorePickup3);
                    setPathState(13);
                    opmodeTimer.resetTimer();
                }
                break;

            case 13:

                if (isCloseTo(scorePose)) {
                    clawSubsystem.release();
                    clawSubsystem.grab();
                    clawSubsystem.moveDown(); // Release object
                    opmodeTimer.resetTimer();
                    setPathState(-1);
                }
                else if (opmodeTimer.getElapsedTimeSeconds() > 4) {
                    clawSubsystem.moveDown();
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

    private boolean isCloseTo(Pose targetPose) {
        double tolerance = 1;
        // Allowable distance error (tolerance)

        double distance = Math.sqrt(Math.pow(follower.getPose().getX() - targetPose.getX(), 2)
                + Math.pow(follower.getPose().getY() - targetPose.getY(), 2));
        return distance < tolerance;
    }
}
