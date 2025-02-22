package org.firstinspires.ftc.teamcode.Regionals;

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
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Autonomous(name = "AutoRightSide_pickanddump", group = "Competition")
public class AutoTestRightSide_V4_20250202_PickandDumpSamples extends OpMode {
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


    // afterSpecimanPose is the end of the curve
    private Pose placeSpecimenPose1 = new Pose(29, 15, 0);


    private final Pose specimen1PickPose = new Pose(17.75, -33., Math.toRadians(180));
    private final Pose specimen2PickPose = new Pose(17, -44, Math.toRadians(180));
 //   private final Pose specimen3PickPose = new Pose(36, -34.4, Math.toRadians(180));

    private final Pose readyForHuman = new Pose(15, -21.8, Math.toRadians(180));
    private final Pose humanPlayerZonePose = new Pose(-8, -21.8, Math.toRadians(180));
    private final Pose humanPlayerZonebackPose = new Pose(-1, -21.8, Math.toRadians(180));
    private final Pose highChamberPose = new Pose(0, 14, 0);
    private final Pose returnHomePose = new Pose(-2, 0, 0);

    // Paths
    private Path pickSpiciment1Path, placeSpecimenPath,
    readyForHumanPath, getSpeciPath, getSpeciBackPath, specimenToHighChamberPath, returnHomePath;
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
        placeSpecimenPath = new Path(new BezierLine(new Point(startPose), new Point(placeSpecimenPose)));
        placeSpecimenPath.setLinearHeadingInterpolation(startPose.getHeading(), placeSpecimenPose.getHeading());

        pickSpiciment1Path = new Path(new BezierLine(new Point(placeSpecimenPose), new Point(specimen1PickPose)));
        pickSpiciment1Path.setLinearHeadingInterpolation(placeSpecimenPose.getHeading(), specimen1PickPose.getHeading());

        readyForHumanPath = new Path(new BezierLine(new Point(placeSpecimenPose1), new Point(readyForHuman)));
        readyForHumanPath.setLinearHeadingInterpolation(placeSpecimenPose1.getHeading(), readyForHuman.getHeading());



        getSpeciPath = new Path(new BezierLine(new Point(readyForHuman), new Point(humanPlayerZonePose)));
        getSpeciPath.setLinearHeadingInterpolation(readyForHuman.getHeading(), humanPlayerZonePose.getHeading());

        getSpeciBackPath = new Path(new BezierLine(new Point(humanPlayerZonePose), new Point(humanPlayerZonebackPose)));
        getSpeciBackPath.setLinearHeadingInterpolation(humanPlayerZonePose.getHeading(), humanPlayerZonebackPose.getHeading());

        specimenToHighChamberPath = new Path(new BezierLine(new Point(humanPlayerZonebackPose), new Point(placeSpecimenPose1)));
        specimenToHighChamberPath.setLinearHeadingInterpolation(humanPlayerZonebackPose.getHeading(), highChamberPose.getHeading());

        returnHomePath = new Path(new BezierLine(new Point(highChamberPose), new Point(returnHomePose)));
        returnHomePath.setLinearHeadingInterpolation(highChamberPose.getHeading(), returnHomePose.getHeading());

        //working to before first push part
        //pushPath = new PathChain(specimen1Path, specimen1StrafePath);

     //   pushPath = new PathChain(specimen1Path, specimen1StrafePath, specimen1PushPath, specimen2Path, specimen2StrafePath, specimen2PushPath, readyForHumanPath1);
        //pushPath = new PathChain(specimen1Path, specimen1StrafePath, specimen1PushPath, specimen2Path, specimen2StrafePath, specimen2PushPath, readyForHumanPath1);
     //   pushSpecimen1Path = new PathChain(specimen1ReadyPushPath, specimen1Path);
        //ww    pushSpecimen1Path = new PathChain(specimen1ReadyPushPath);
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Initial state
                if (clawSubsystem.height == 0) {
                    follower.followPath(placeSpecimenPath, true);
                    clawSubsystem.moveUpSpReadyToRelease();
                    clawSubsystem.grabVertical();

                } else if (clawSubsystem.height == 200) {
                    setPathState(1);
                }
                break;
            case 1: // Place specimen
                if (isNearPose(follower.getPose(), placeSpecimenPose, 1)) {
                    if (clawSubsystem.height == 200) {
                        clawSubsystem.moveDownSpToRelease();
                        // elevatorUp = false;
                        // opmodeTimer.resetTimer();
                        // setPathState(-1);
                    }
                }

                if (clawSubsystem.height == 300) {
                        // if (opmodeTimer.getElapsedTimeSeconds() > 0.5) {
                        follower.followPath(pickSpiciment1Path, true);

                        // Move to a point and turn direction
                        //clawSubsystem.moveDown();
                        setPathState(2);
                }
                break;


            case 2: // pick and dump sample # 1


                if (isNearPose(follower.getPose(), specimen1PickPose, 1)) {
                    if (clawSubsystem.height != 0) {
                        //clawSubsystem.moveDown();
                    }
                    clawSubsystem.pickandDumpRightSamples();
                    setPathState(-1);
                }
                break;
/*

            case 2: // Push First sample, and stop

                if (isNearPose(follower.getPose(), afterSpecimenPose, 1)) {
                    // clawSubsystem.moveDownSp();
                    clawSubsystem.moveToPickingReady();
                    follower.followPath(pushPath, true);
                    opmodeTimer.resetTimer();
                    setPathState(5);
                }
                break;
*/
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
                    follower.followPath(getSpeciPath, true);
                    opmodeTimer.resetTimer();
                    // clawSubsystem.height = 0;
                    //clawSubsystem.moveUpSp();
                    setPathState(12);
                }
                break;
            case 12: // Back to play area
                if (opmodeTimer.getElapsedTimeSeconds() > 2) {


                    if (clawSubsystem.height == 1) {
                        follower.followPath(getSpeciBackPath, true);
                        x += 3;
                            clawSubsystem.moveUpSp();
                            follower.followPath(specimenToHighChamberPath, true);

                    }

                    else {
                        if (opmodeTimer.getElapsedTimeSeconds() > 1) {
                            telemetry.addData("Elevator Position", clawSubsystem.getElevatorPosition());
                            if (clawSubsystem.getElevatorPosition() >= -1000) {
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
                                follower.followPath(specimenToHighChamberPath, true);
                                opmodeTimer.resetTimer();
                                setPathState(13);
                            }
                            telemetry.update();
                        }
                    }
                }
                break;

            case 13: // High chamber action
                if (isNearPose(follower.getPose(), placeSpecimenPose1, 1)) {
                    if (opmodeTimer.getElapsedTimeSeconds()>0) {
                        clawSubsystem.moveDownSpToRelease();
                    }


                        // if (opmodeTimer.getElapsedTimeSeconds() > 0.5) {
                    if (opmodeTimer.getElapsedTimeSeconds()>3) {
                        follower.followPath(readyForHumanPath, true);
                        opmodeTimer.resetTimer();
                        setPathState(14);


                    }

                }


                break;
            case 14: // Ready for human interaction
                if (isNearPose(follower.getPose(), readyForHuman, 1)) {
                    follower.followPath(getSpeciPath, true);
                    opmodeTimer.resetTimer();
                    // clawSubsystem.height = 0;
                    //clawSubsystem.moveUpSp();
                    setPathState(16);
                }
                break;
            case 16: // Back to play area
                if (opmodeTimer.getElapsedTimeSeconds() > 2) {


                    if (clawSubsystem.height == 1) {
                        follower.followPath(getSpeciBackPath, true);
                        x += 3;
                        clawSubsystem.moveUpSp();
                        follower.followPath(specimenToHighChamberPath, true);

                    }

                    else {
                        if (opmodeTimer.getElapsedTimeSeconds() > 1) {
                            telemetry.addData("Elevator Position", clawSubsystem.getElevatorPosition());
                            if (clawSubsystem.getElevatorPosition() >= -1000) {
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
                                follower.followPath(specimenToHighChamberPath, true);
                                opmodeTimer.resetTimer();
                                setPathState(17);
                            }
                            telemetry.update();
                        }
                    }
                }
                break;

            case 18: // High chamber action
                if (isNearPose(follower.getPose(), placeSpecimenPose1, 1)) {
                    clawSubsystem.moveDownSpToRelease();
                    if (opmodeTimer.getElapsedTimeSeconds()>3){

                        // if (opmodeTimer.getElapsedTimeSeconds() > 0.5) {
 //                       follower.followPath(pushReadyPath, true);
                        opmodeTimer.resetTimer();
                        setPathState(19);


                    }

                }






                break;
            case 19: // Ready for human interaction
                if (isNearPose(follower.getPose(), readyForHuman, 1)) {
                    follower.followPath(getSpeciPath, true);
                    opmodeTimer.resetTimer();
                    // clawSubsystem.height = 0;
                    //clawSubsystem.moveUpSp();
                    setPathState(20);
                }


                break;

        case 20:
        if (isNearPose(follower.getPose(), highChamberPose, 1.5)) {
            follower.followPath(returnHomePath, true);
            setPathState(-1); // End
        }
        break;

        case -1: // Autonomous complete
        telemetry.addData("Status", "Autonomous Complete");
        break;

    }
}

    private boolean isNearPose(Pose current, Pose target, double tolerance) {
        return Math.abs(current.getX() - target.getX()) < tolerance &&
                Math.abs(current.getY() - target.getY()) < tolerance;
    }

    private void setPathState(int state) {
        pathState = state;
    }
}
