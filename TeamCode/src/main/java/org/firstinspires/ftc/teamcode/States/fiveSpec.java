package org.firstinspires.ftc.teamcode.States;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Autonomous(name = "Get_Shitt_OnV2", group = "Competition")
public class fiveSpec extends OpMode {
    private int x = 0;
    private int pathState = 0;
    private int highChamberCount = 0;
    private boolean elevatorUp = false;

    private Follower follower;
    private Timer opmodeTimer;
    private ClawSubsystem clawSubsystem;

    // Starting and target positions
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private Pose placeSpecimenPose = new Pose(30, 12, 0);

    // Change push Ready Path as a curve
    //        pushReadyPath , this is for the 1st speciman push with a curve path


    // afterSpecimanPose is the end of the curve
    private Pose placeSpecimenPose1 = new Pose(29.5, 16, 0);
    private Pose placeSpecimenPose2 = new Pose(29.5, 15, 0);
    private Pose placeSpecimenPose3 = new Pose(29.5, 14.5, 0);
    private Pose placeSpecimenPose4 = new Pose(29.5, 14, 0);
    //    private final Pose specimenReayControlPose = new Pose(5, -25, 0);
//    private final Pose afterSpecimenPose = new Pose(52, -18, Math.toRadians(180));


    private final Pose afterSpecimenPose = new Pose(20, -17.4, Math.toRadians(180));

    private final Pose specimen1ControlPose = new Pose(60, -26, Math.toRadians(180));
    private final Pose specimen1PushReadyPose = new Pose(8.5, -14.2, Math.toRadians(180));
    private final Pose specimen1Pose = new Pose(45, -25.4, Math.toRadians(180));
    private final Pose specimen1StrafePose = new Pose(48, -29, Math.toRadians(180));
    private final Pose specimen1PushPose = new Pose(25, -29, Math.toRadians(180));
    private final Pose specimen2Pose = new Pose(45, -36, Math.toRadians(180));
    private final Pose specimen2StrafePose = new Pose(48, -40, Math.toRadians(180));
    private final Pose specimen2PushPose = new Pose(29, -40, Math.toRadians(180));

    private final Pose specimen3Pose = new Pose(45, -42, Math.toRadians(180));
    private final Pose specimen3StrafePose = new Pose(48, -46, Math.toRadians(180));
    private final Pose specimen3PushPose = new Pose(29, -46, Math.toRadians(180));

    private final Pose readyForHuman = new Pose(10, -21.8, Math.toRadians(180));
    private final Pose humanPlayerZonePose = new Pose(-5, -21.8, Math.toRadians(180));
    private final Pose humanPlayerZonebackPose = new Pose(0, -21.8, Math.toRadians(180));
    private final Pose highChamberPose = new Pose(0, 14, 0);
    private final Pose returnHomePose = new Pose(25, 15, 0);
    // Paths
    private Path specimenToHighChamberPath1, specimenToHighChamberPath2, specimenToHighChamberPath3, specimenToHighChamberPath4, placeSpecimenPath, pushReadyPath, specimen1ReadyPushPath,specimen1Path, specimen1StrafePath,
            specimen1PushPath, specimen2Path, specimen2StrafePath, specimen2PushPath,specimen3Path, specimen3StrafePath, specimen3PushPath,
            readyForHumanPath1,readyForHumanPath, getSpeciPath, getSpeciBackPath, specimenToHighChamberPath, returnHomePath;
    private PathChain pushPath, pushSpecimen1Path;

    @Override
    public void init() {
        clawSubsystem = new ClawSubsystem(hardwareMap);
        follower = new Follower(hardwareMap);
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower.setStartingPose(startPose);
        //clawSubsystem.height = 0;
        buildPaths();
        clawSubsystem.moveDownSp();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();

        if (clawSubsystem.height != 0) {
            clawSubsystem.moveDownSp();
        }

    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        Telemetry.Item elapsedTime = telemetry.addData("Elapsed Time", opmodeTimer.getElapsedTimeSeconds());

        telemetry.addData("elevator Height", clawSubsystem.height);
        telemetry.addData("elevator Current Positon", clawSubsystem.getElevatorPosition());
        telemetry.addData("isNearPose(follower.getPose(), placeSpecimenPose, 1)", isNearPose(follower.getPose(), placeSpecimenPose, 1));
        telemetry.update();
    }

    private void buildPaths() {
        FollowerConstants.speed = 1;
        placeSpecimenPath = new Path(new BezierLine(new Point(startPose), new Point(placeSpecimenPose)));
        placeSpecimenPath.setLinearHeadingInterpolation(startPose.getHeading(), placeSpecimenPose.getHeading());

        pushReadyPath = new Path(new BezierLine(new Point(placeSpecimenPose), new Point(afterSpecimenPose)));
        pushReadyPath.setLinearHeadingInterpolation(placeSpecimenPose.getHeading(), afterSpecimenPose.getHeading());


// Change push Ready Path as a curve

//        pushReadyPath = new Path(new BezierCurve(new Point(placeSpecimenPose), new Point(specimenReayControlPose), new Point(afterSpecimenPose)));
//        pushReadyPath.setLinearHeadingInterpolation(placeSpecimenPose.getHeading(), afterSpecimenPose.getHeading());


        specimen1ReadyPushPath = new Path(new BezierCurve(new Point(afterSpecimenPose), new Point(specimen1ControlPose), new Point(specimen1PushReadyPose)));
        specimen1ReadyPushPath.setLinearHeadingInterpolation(afterSpecimenPose.getHeading(), specimen1PushReadyPose.getHeading());


//        specimen1Path = new Path(new BezierLine(new Point(afterSpecimenPose), new Point(specimen1Pose)));
//        specimen1Path.setLinearHeadingInterpolation(afterSpecimenPose.getHeading(), specimen1Pose.getHeading());

        specimen1Path = new Path(new BezierLine(new Point(specimen1PushReadyPose), new Point(specimen1Pose)));
        specimen1Path.setLinearHeadingInterpolation(afterSpecimenPose.getHeading(), specimen1Pose.getHeading());


        specimen1StrafePath = new Path(new BezierLine(new Point(specimen1Pose), new Point(specimen1StrafePose)));
        specimen1StrafePath.setLinearHeadingInterpolation(specimen1Pose.getHeading(), specimen1StrafePose.getHeading());

        specimen1PushPath = new Path(new BezierLine(new Point(specimen1StrafePose), new Point(specimen1PushPose)));
        specimen1PushPath.setLinearHeadingInterpolation(specimen1StrafePose.getHeading(), specimen1PushPose.getHeading());

        specimen2Path = new Path(new BezierLine(new Point(specimen1PushPose), new Point(specimen2Pose)));
        specimen2Path.setLinearHeadingInterpolation(specimen1PushPose.getHeading(), specimen2Pose.getHeading());

        specimen2StrafePath = new Path(new BezierLine(new Point(specimen2Pose), new Point(specimen2StrafePose)));
        specimen2StrafePath.setLinearHeadingInterpolation(specimen2Pose.getHeading(), specimen2StrafePose.getHeading());

        specimen2PushPath = new Path(new BezierLine(new Point(specimen2StrafePose), new Point(specimen2PushPose)));
        specimen2PushPath.setLinearHeadingInterpolation(specimen2StrafePose.getHeading(), specimen2PushPose.getHeading());

        specimen3Path = new Path(new BezierLine(new Point(specimen3PushPose), new Point(specimen3Pose)));
        specimen3Path.setLinearHeadingInterpolation(specimen3PushPose.getHeading(), specimen3Pose.getHeading());

        specimen3StrafePath = new Path(new BezierLine(new Point(specimen3Pose), new Point(specimen3StrafePose)));
        specimen3StrafePath.setLinearHeadingInterpolation(specimen3Pose.getHeading(), specimen3StrafePose.getHeading());

        specimen3PushPath = new Path(new BezierLine(new Point(specimen3StrafePose), new Point(specimen3PushPose)));
        specimen3PushPath.setLinearHeadingInterpolation(specimen3StrafePose.getHeading(), specimen3PushPose.getHeading());

        readyForHumanPath = new Path(new BezierLine(new Point(placeSpecimenPose1), new Point(readyForHuman)));
        readyForHumanPath.setLinearHeadingInterpolation(placeSpecimenPose1.getHeading(), readyForHuman.getHeading());


        readyForHumanPath1 = new Path(new BezierLine(new Point(specimen2PushPose), new Point(readyForHuman)));
        readyForHumanPath1.setLinearHeadingInterpolation(specimen2PushPose.getHeading(), readyForHuman.getHeading());

        getSpeciPath = new Path(new BezierLine(new Point(readyForHuman), new Point(humanPlayerZonePose)));
        getSpeciPath.setLinearHeadingInterpolation(readyForHuman.getHeading(), humanPlayerZonePose.getHeading());

        getSpeciBackPath = new Path(new BezierLine(new Point(humanPlayerZonePose), new Point(humanPlayerZonebackPose)));
        getSpeciBackPath.setLinearHeadingInterpolation(humanPlayerZonePose.getHeading(), humanPlayerZonebackPose.getHeading());

        specimenToHighChamberPath1 = new Path(new BezierLine(new Point(startPose), new Point(placeSpecimenPose1)));
        specimenToHighChamberPath1.setLinearHeadingInterpolation(startPose.getHeading(), highChamberPose.getHeading());

        specimenToHighChamberPath2 = new Path(new BezierLine(new Point(humanPlayerZonebackPose), new Point(placeSpecimenPose2)));
        specimenToHighChamberPath2.setLinearHeadingInterpolation(humanPlayerZonebackPose.getHeading(), highChamberPose.getHeading());

        specimenToHighChamberPath3 = new Path(new BezierLine(new Point(humanPlayerZonebackPose), new Point(placeSpecimenPose3)));
        specimenToHighChamberPath3.setLinearHeadingInterpolation(humanPlayerZonebackPose.getHeading(), highChamberPose.getHeading());

        specimenToHighChamberPath4 = new Path(new BezierLine(new Point(humanPlayerZonebackPose), new Point(placeSpecimenPose4)));
        specimenToHighChamberPath4.setLinearHeadingInterpolation(humanPlayerZonebackPose.getHeading(), highChamberPose.getHeading());

        returnHomePath = new Path(new BezierLine(new Point(highChamberPose), new Point(returnHomePose)));
        returnHomePath.setLinearHeadingInterpolation(highChamberPose.getHeading(), returnHomePose.getHeading());

        //working to before first push part
        //pushPath = new PathChain(specimen1Path, specimen1StrafePath);

        pushPath = new PathChain(pushReadyPath, specimen1Path, specimen1StrafePath, specimen1PushPath, specimen2Path, specimen2StrafePath, specimen2PushPath, specimen3Path, specimen3StrafePath,specimen3PushPath, readyForHumanPath1);
        //pushPath = new PathChain(specimen1Path, specimen1StrafePath, specimen1PushPath, specimen2Path, specimen2StrafePath, specimen2PushPath, readyForHumanPath1);
        pushSpecimen1Path = new PathChain(specimen1ReadyPushPath, specimen1Path);
        //ww    pushSpecimen1Path = new PathChain(specimen1ReadyPushPath);
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Initial state
                if (clawSubsystem.getElevatorPosition() >= -100) {
                    clawSubsystem.moveUpSpReadyToRelease();
                } else {
                    follower.followPath(specimenToHighChamberPath1, true);
                    setPathState(1);
                    clawSubsystem.grabVertical();
                }
                break;
            case 1: // Place specimen
                if (isNearPose(follower.getPose(), placeSpecimenPose1, 1.3)) {
                    if (clawSubsystem.height == 200) {
                        clawSubsystem.moveDownSpToRelease();
                        setPathState(2);

                        // elevatorUp = false;
                        // opmodeTimer.resetTimer();
                        // setPathState(-1);
                    }
                }
/*
                if (clawSubsystem.height == 300) {
                    // if (opmodeTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(pushReadyPath, true);

                    // Move to a point and turn direction
                    setPathState(2);
                }

 */
                break;

            case 2: // Push First sample, and stop

                if (isNearPose(follower.getPose(), placeSpecimenPose, 1)) {
                    // clawSubsystem.moveDownSp();
                    clawSubsystem.moveToPickingReady();
                    follower.followPath(pushPath, true);
                    opmodeTimer.resetTimer();
                    setPathState(5);
                }
                break;

// Stop here
            /*
            case 3: // Ready for next phase
                if (isNearPose(follower.getPose(), readyForHuman, 1)) {

                    // clawSubsystem.moveDownSp();
                    //follower.followPath(pushPath, true);
                    follower.followPath(pushSpecimen1Path, true);
                    opmodeTimer.resetTimer();
                    setPathState(5);
                }
                break;
                */

            // Stop at the back position

            case 5: // Ready for human interaction
                if (isNearPose(follower.getPose(), readyForHuman, 1)) {
                    clawSubsystem.moveToPickingReady();
                    follower.followPath(getSpeciPath, true);
                    opmodeTimer.resetTimer();
                    // clawSubsystem.height = 0;
                    //clawSubsystem.moveUpSp();
                    setPathState(12);
                }
                break;
            case 12: // Back to play area
                if (opmodeTimer.getElapsedTimeSeconds() > 0) {


                    if (clawSubsystem.height == 1) {
                        follower.followPath(getSpeciBackPath, true);
                        x += 3;
                        clawSubsystem.moveUpSp();
                        follower.followPath(specimenToHighChamberPath2, true);

                    }

                    else {
                        if (opmodeTimer.getElapsedTimeSeconds() > 0.6) {
                            telemetry.addData("Elevator Position", clawSubsystem.getElevatorPosition());
                            if (clawSubsystem.getElevatorPosition() >= -500) {
                                telemetry.addData("Status", "Elevator moving up");
                                clawSubsystem.moveUpSp();
                            } else {
                                telemetry.addData("Status", "Following back path");
                                follower.followPath(getSpeciBackPath, true);

                                //follower.followPath(getSpeciBackPath, true);
                                x += 3;
//                            placeSpecimenPose1 = new Pose(32, 23 - x, 0);
                                specimenToHighChamberPath = new Path(new BezierLine(new Point(humanPlayerZonebackPose), new Point(placeSpecimenPose1)));
                                specimenToHighChamberPath.setLinearHeadingInterpolation(humanPlayerZonebackPose.getHeading(), placeSpecimenPose1.getHeading());
                                follower.followPath(specimenToHighChamberPath2, true);
                                opmodeTimer.resetTimer();
                                setPathState(13);
                            }
                            telemetry.update();
                        }
                    }
                }
                break;

            case 13: // High chamber action
                if (isNearPose(follower.getPose(), placeSpecimenPose2, 1.3)) {
                    if (opmodeTimer.getElapsedTimeSeconds()>0) {
                        clawSubsystem.moveDownSpToRelease();
                    }


                    // if (opmodeTimer.getElapsedTimeSeconds() > 0.5) {
                    if (opmodeTimer.getElapsedTimeSeconds()>3) {
                        //clawSubsystem.moveToPickingReady();

                        follower.followPath(readyForHumanPath, true);
                        opmodeTimer.resetTimer();
                        setPathState(14);


                    }

                }
                break;

            case 14: // Ready for human interaction


                if (isNearPose(follower.getPose(), readyForHuman, 20)){
                    clawSubsystem.moveToPickingReady();
                }

                if (isNearPose(follower.getPose(), readyForHuman, 1)) {
                    clawSubsystem.moveToPickingReady();

                    follower.followPath(getSpeciPath, true);
                    opmodeTimer.resetTimer();
                    // clawSubsystem.height = 0;
                    //clawSubsystem.moveUpSp();
                    setPathState(16);
                }
                break;
            case 16: // Back to play area
                if (opmodeTimer.getElapsedTimeSeconds() > 0 ) {


                    if (clawSubsystem.height == 1) {
                        follower.followPath(getSpeciBackPath, true);
                        x += 3;
                        clawSubsystem.moveUpSp();
                        follower.followPath(specimenToHighChamberPath3, true);

                    }

                    else {
                        if (opmodeTimer.getElapsedTimeSeconds() > 0.3) {
                            telemetry.addData("Elevator Position", clawSubsystem.getElevatorPosition());
                            if (clawSubsystem.getElevatorPosition() >= -400) {
                                telemetry.addData("Status", "Elevator moving up");
                                clawSubsystem.moveUpSp();
                            } else {
                                telemetry.addData("Status", "Following back path");
                                follower.followPath(getSpeciBackPath, true);

                                //follower.followPath(getSpeciBackPath, true);
                                x += 3;
//                            placeSpecimenPose1 = new Pose(32, 23 - x, 0);
                                specimenToHighChamberPath = new Path(new BezierLine(new Point(humanPlayerZonebackPose), new Point(placeSpecimenPose1)));
                                specimenToHighChamberPath.setLinearHeadingInterpolation(humanPlayerZonebackPose.getHeading(), placeSpecimenPose1.getHeading());
                                follower.followPath(specimenToHighChamberPath3, true);
                                opmodeTimer.resetTimer();
                                setPathState(18);
                            }
                            telemetry.update();
                        }
                    }
                }
                break;

            case 18: // High chamber action
                if (isNearPose(follower.getPose(), placeSpecimenPose3, 1.3)) {
                    clawSubsystem.moveDownSpToRelease();
                    if (opmodeTimer.getElapsedTimeSeconds()>3){

                        // if (opmodeTimer.getElapsedTimeSeconds() > 0.5) {
                        //clawSubsystem.moveToPickingReady();

                        follower.followPath(readyForHumanPath, true);
                        opmodeTimer.resetTimer();
                        setPathState(20);


                    }

                }






                break;
            case 20:
                if (isNearPose(follower.getPose(), readyForHuman, 20)){
                    clawSubsystem.moveToPickingReady();
                }

                // Ready for human interaction
                if (isNearPose(follower.getPose(), readyForHuman, 1)) {
                    //    clawSubsystem.moveToPickingReady();
                    follower.followPath(getSpeciPath, true);
                    opmodeTimer.resetTimer();
                    // clawSubsystem.height = 0;
                    //clawSubsystem.moveUpSp();
                    setPathState(21);
                }
                break;
            case 21: // Back to play area
                if (opmodeTimer.getElapsedTimeSeconds() > 0.8) {


                    if (clawSubsystem.height == 1) {
                        follower.followPath(getSpeciBackPath, true);
                        x += 3;
                        clawSubsystem.moveUpSp();
                        follower.followPath(specimenToHighChamberPath4, true);

                    }

                    else {
                        if (opmodeTimer.getElapsedTimeSeconds() > 0.2) {
                            telemetry.addData("Elevator Position", clawSubsystem.getElevatorPosition());
                            if (clawSubsystem.getElevatorPosition() >= -400) {
                                telemetry.addData("Status", "Elevator moving up");
                                clawSubsystem.moveUpSp();
                            } else {
                                telemetry.addData("Status", "Following back path");
                                follower.followPath(getSpeciBackPath, true);

                                //follower.followPath(getSpeciBackPath, true);
                                x += 3;
//                            placeSpecimenPose1 = new Pose(32, 23 - x, 0);
                                specimenToHighChamberPath = new Path(new BezierLine(new Point(humanPlayerZonebackPose), new Point(placeSpecimenPose1)));
                                specimenToHighChamberPath.setLinearHeadingInterpolation(humanPlayerZonebackPose.getHeading(), placeSpecimenPose1.getHeading());
                                follower.followPath(specimenToHighChamberPath4, true);
                                opmodeTimer.resetTimer();
                                setPathState(22);
                            }
                            telemetry.update();
                        }
                    }
                }
                break;

            case 22: // High chamber action
                if (isNearPose(follower.getPose(), placeSpecimenPose4, 1.3)) {
                    clawSubsystem.moveDownSpToRelease();


                    //clawSubsystem.extend();
                    //clawSubsystem.grabReady();



                    opmodeTimer.resetTimer();
                    setPathState(23);







                }






                break;




            case 23:
                if (opmodeTimer.getElapsedTimeSeconds()>0.5) {
                    follower.followPath(returnHomePath, true);

                /*
                if (clawSubsystem.getElevatorPosition() >= -100) {


                    follower.followPath(returnHomePath, true);
                    //follower.followPath(returnHomePath, true);
                    clawSubsystem.moveDown();
                }
                else{
                    follower.followPath(returnHomePath, true);
                }
            }

                */
                    setPathState(25); // End
                }

                break;
            case 25:
                if (isNearPose(follower.getPose(), returnHomePose, 3)){
                    clawSubsystem.grabVertical();
                    clawSubsystem.moveDown();
                    //setPathState(-1);

                }

            case -1: // Autonomous complete
                telemetry.addData("Status", "Autonomous Complete");
                break;

        }
    }

    private boolean isNearPose(Pose current, Pose target,    double tolerance) {
        return Math.abs(current.getX() - target.getX()) < 2*tolerance &&
                Math.abs(current.getY() - target.getY()) < 2*tolerance;
    }

    private void setPathState(int state) {
        pathState = state;
    }
}
