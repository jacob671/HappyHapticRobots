
/*
package org.firstinspires.ftc.teamcode.Regionals;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

package org.firstinspires.ftc.teamcode.States;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.config.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Autonomous(name = "AutoLeftSide", group = "Examples")
public class AutoTest3 extends OpMode {

    private Follower follower;
    private Timer opmodeTimer;
    private int pathState = 0;
    private int loopNumber = 0;

    private double positionTol = 1;
    // position tolerance for the robot


    // private int caseNumber = 0;
    // Starting and target positions
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose scorePose = new Pose(6, 18, 2.37);
    private final Pose pickup1Pose = new Pose(16, 9.5, 3.15);
    private final Pose pickup2Pose = new Pose(16, 17.98, Math.toRadians(180));
    private final Pose pickup3Pose = new Pose(35., 10.5, 4.7);
    private final Pose parkPose = new Pose(65.3, -14, 1.56);
    // Paths and subsystems
    private Path scorePreload;
    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3, park;
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

        park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(parkPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();

    }


    private void autonomousPathUpdate() {
        switch (pathState) {


            case 0:

                if (clawSubsystem.getElevatorPosition() >= -1200) {
                    clawSubsystem.moveUp();
                } else {
                    follower.followPath(scorePreload, true);
                    opmodeTimer.resetTimer();
                    setPathState(1);
                    //telemetry.addData("PathState", "1");
                }
                break;
            case 1:
                // Dump bucket object and move to next position
                if (isNearPose(follower.getPose(), scorePose, 1.2)) {
                    /*

                    if (sampleInBasket) {
                        if (clawSubsystem.height != 2) {
                            clawSubsystem.moveUp();
                        }
                        else {


                    // elevator is at high position, release bucket object
                    clawSubsystem.bucket();
                    //clawSubsystem.extend();
                    sampleInBasket = false;
                    opmodeTimer.resetTimer();
                    setPathState(2);


                }
                break;

/*
            case 15: // Back to play area
                if (opmodeTimer.getElapsedTimeSeconds() > 2) {
                    if (opmodeTimer.getElapsedTimeSeconds() > 1) {
                        telemetry.addData("Elevator Position", clawSubsystem.getElevatorPosition());
                        if (clawSubsystem.getElevatorPosition() >= -1000) {
                            clawSubsystem.moveUpSp();
                        } else {

                            clawSubsystem.bucket();
                            sampleInBasket = false;
                            opmodeTimer.resetTimer();
                            setPathState(2);
                        }

                        telemetry.update();

                    }
                }
                break;


  */
/*
            case 2:
// Lower elevator, follow path, and release claw object
                if (clawSubsystem.height != 0 && opmodeTimer.getElapsedTimeSeconds() > 0.8) {
                    clawSubsystem.grabReady();
                    //clawSubsystem.moveDown();
                    follower.followPath(grabPickup1, true);
                    opmodeTimer.resetTimer();
                    setPathState(3);
                }
                break;
            case 3:
                // picking up the first sample
                if (isNearPose(follower.getPose(), pickup1Pose, 1.3)) {

                    if (clawSubsystem.getElevatorPosition() <= -3900) {
                        clawSubsystem.moveDown();
                        clawSubsystem.grabReady();
                    } else {
                        if (!sampleInBasket) {

                            clawSubsystem.grab();
                            if (opmodeTimer.getElapsedTimeSeconds() > 2.5) {

                                clawSubsystem.release();
                                sampleInBasket = true;
                                opmodeTimer.resetTimer();
                                setPathState(4);
                            }

                        }
                    }
                }
                break;

            case 4:
                // move first sample to dumping place
                if (clawSubsystem.getElevatorPosition() >= -1400) {
                    if (opmodeTimer.getElapsedTimeSeconds() > 1) {

                        clawSubsystem.moveUp();
                    }
                } else {
                    follower.followPath(scorePickup1, true);
                    opmodeTimer.resetTimer();
                    setPathState(5);
                    //telemetry.addData("PathState", "1");
                }
                break;
            case 5:
                // Dump bucket object and move to next position, like case 1
                if (isNearPose(follower.getPose(), scorePose, 1.2)) {
                    /*

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
                break;


            case 6:
                if (clawSubsystem.height != 0 && opmodeTimer.getElapsedTimeSeconds() > 1) {
                    clawSubsystem.grabReady();
                    //clawSubsystem.moveDown();
                    follower.followPath(grabPickup2, true);
                    opmodeTimer.resetTimer();
                    setPathState(7);
                }
                break;
            case 7:
                if (isNearPose(follower.getPose(), pickup2Pose, 1)) {

                    if (clawSubsystem.getElevatorPosition() <= -3900) {
                        clawSubsystem.moveDown();
                        clawSubsystem.grabReady();
                    } else {
                        if (!sampleInBasket) {

                            clawSubsystem.grab();
                            if (opmodeTimer.getElapsedTimeSeconds() > 2.5) {
                                clawSubsystem.release();
                                sampleInBasket = true;
                                opmodeTimer.resetTimer();
                                setPathState(8);
                            }

                        }
                    }
                }
                break;

            case 8:
                if (clawSubsystem.getElevatorPosition() >= -1400) {
                    if (opmodeTimer.getElapsedTimeSeconds() > 1) {

                        clawSubsystem.moveUp();
                    }
                } else {
                    follower.followPath(scorePickup2, true);
                    opmodeTimer.resetTimer();
                    setPathState(9);
                    //telemetry.addData("PathState", "1");
                }
                break;

            case 9:
                // Dump bucket object and move to next position, like case 1
                if (isNearPose(follower.getPose(), scorePose, 1.2)) {
                    /*

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
                break;


            case 10:
                if (clawSubsystem.height != 0 && opmodeTimer.getElapsedTimeSeconds() > 1) {
                    clawSubsystem.grabVertical();
                    follower.followPath(grabPickup3, true);
                    opmodeTimer.resetTimer();
                    setPathState(11);
                }
                break;

            case 11:
                if (isNearPose(follower.getPose(), pickup3Pose, 1)) {
                    if (clawSubsystem.getElevatorPosition() <= -1900) {
                        clawSubsystem.moveDown();
                        clawSubsystem.grabReady();
                    } else {
                        if (!sampleInBasket) {
                            clawSubsystem.grab();
                            if (opmodeTimer.getElapsedTimeSeconds() > 3.5) {
                                clawSubsystem.release();
                                sampleInBasket = true;
                                opmodeTimer.resetTimer();
                                setPathState(12);
                            }
                        }
                    }
                }
                break;

            case 12:
                if (clawSubsystem.getElevatorPosition() >= -1400) {
                    if (opmodeTimer.getElapsedTimeSeconds() > 1) {
                        clawSubsystem.moveUp();
                    }
                } else {
                    follower.followPath(scorePickup3, true);
                    opmodeTimer.resetTimer();
                    setPathState(13);
                }
                break;

            case 13:
                if (isNearPose(follower.getPose(), scorePose, 1.2)) {
                    clawSubsystem.bucket();
                    sampleInBasket = false;
                    opmodeTimer.resetTimer();
                    setPathState(14);
                }
                break;

            case 14:
                if (clawSubsystem.height != 0 && opmodeTimer.getElapsedTimeSeconds() > 1) {
                    if (clawSubsystem.getElevatorPosition() <= -3000) {
                        clawSubsystem.moveDown();
                        clawSubsystem.extendOriginal();
                    } else {
                        follower.followPath(park, true);
                        opmodeTimer.resetTimer();
                        setPathState(15);
                    }
                }
                break;

            case 15:
                if (isNearPose(follower.getPose(), parkPose, positionTol)) {
                    if (clawSubsystem.height != 0) {
                        clawSubsystem.moveDown();
                        clawSubsystem.grabReady();
                    }
                    opmodeTimer.resetTimer();
                    setPathState(16);
                }
                break;

            case 16:
                detectAndMoveToYellowBlock();
                break;
        }

        // Telemetry updates
        telemetry.addData("Path State", getPathState());
        telemetry.addData("Robot Position", follower.getPose().toString());
        telemetry.addData("Elevator Position", clawSubsystem.getElevatorPosition());
        telemetry.addData("Claw Height", clawSubsystem.height);
        telemetry.addData("Sample In Basket", sampleInBasket);
        telemetry.addData("Time Elapsed", opmodeTimer.getElapsedTimeSeconds());
        telemetry.update();
    }


    private void detectAndMoveToYellowBlock() {
        for (int attempt = 0; attempt < 3; attempt++) {
            Mat frame = captureFrame();
            Rect blockRect = detectYellowBlock(frame);
            if (blockRect != null) {
                telemetry.addData("Yellow Block Detected", true);
                telemetry.addData("Block Position", blockRect.toString());
                moveRobotToBlock(blockRect);
                clawSubsystem.grab();
                telemetry.addData("Block Grabbing", "Success");
                scoreBlock();
                telemetry.addData("Block Scoring", "Success");
                setPathState(-1); // Task completed
                return;
            }
            telemetry.addData("Yellow Block Detected", false);
            moveRobotRight();
            telemetry.addData("Attempt", attempt + 1);
            telemetry.update();
        }
        setPathState(-1); // Task failed
        telemetry.addData("Block Scoring", "Failed");
    }

    private Mat captureFrame() {
        Mat frame = new Mat();
        // Implement your camera frame capture logic here
        return frame;
    }

    private Rect detectYellowBlock(Mat frame) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);
        Scalar lowerYellow = new Scalar(20, 100, 100);
        Scalar upperYellow = new Scalar(30, 255, 255);
        Mat mask = new Mat();
        Core.inRange(hsv, lowerYellow, upperYellow, mask);
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        if (!contours.isEmpty()) {
            MatOfPoint c = contours.get(0); // Assuming the largest contour is the block
            return Imgproc.boundingRect(c);
        }
        return null;
    }

    private void moveRobotToBlock(Rect rect) {
        // Implement your logic to move the robot so the camera is above and behind the block
        // Example:
        // if (rect.x < frame.width() / 2) {
        //     // Move left
        //     driveRobot(-0.5, 0.5);
        // } else {
        //     // Move right
        //     driveRobot(0.5, -0.5);
        // }

        // if (rect.y < frame.height() / 2) {
        //     // Move forward
        //     driveRobot(0.5, 0.5);
        // } else {
        //     // Move backward
        //     driveRobot(-0.5, -0.5);
        // }

        // Adjust the logic based on your robot's movement
    }

    private void moveRobotRight() {
        // Implement your logic to move the robot right by one foot
        // Example:
        // driveRobot(0.5, -0.5);
        // sleep(1000); // Adjust the sleep duration as needed
    }

    private void scoreBlock() {
        follower.followPath(scorePickup1, true);
        opmodeTimer.resetTimer();
        setPathState(13); // Go back to state 13 to score the block
    }

    private void grabBlock() {
        clawSubsystem.extend();
        clawSubsystem.grab();
    }


    private int getPathState() {
        // Implement your logic to get the path state
        return 0;
    }

    private void setPathState(int state) {
        pathState = state;
        opmodeTimer.resetTimer();
    }

    private boolean isNearPose(Pose current, Pose target, double tolerance) {
        return (Math.abs(current.getX() - target.getX()) < tolerance &&
                Math.abs(current.getY() - target.getY()) < tolerance);
    }

}

*/