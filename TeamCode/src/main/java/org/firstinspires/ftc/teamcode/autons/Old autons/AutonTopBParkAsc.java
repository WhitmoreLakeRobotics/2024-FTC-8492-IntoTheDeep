package org.firstinspires.ftc.teamcode.autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Settings;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Robot;

@Disabled
@Autonomous(name = "AutonTopBParkAsc", group = "Auton")
// @Autonomous(...) is the other common choice

public class AutonTopBParkAsc extends OpMode {

    //RobotComp robot = new RobotComp();
    Robot robot = new Robot();




    private stage currentStage = stage._unknown;
    // declare auton power variables
    //private double AUTO_DRIVE_TURBO_SPEED = DriveTrain.DRIVETRAIN_TURBOSPEED;
    //private double AUTO_DRIVE_SLOW_SPEED = DriveTrain.DRIVETRAIN_SLOWSPEED;
   // private double AUTO_DRIVE_NORMAL_SPEED = DriveTrain.DRIVETRAIN_NORMALSPEED;
   // private double AUTO_TURN_SPEED = DriveTrain.DRIVETRAIN_TURNSPEED;

    private String RTAG = "8492-Auton";

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        //----------------------------------------------------------------------------------------------
        // These constants manage the duration we allow for callbacks to user code to run for before
        // such code is considered to be stuck (in an infinite loop, or wherever) and consequently
        // the robot controller application is restarted. They SHOULD NOT be modified except as absolutely
        // necessary as poorly chosen values might inadvertently compromise safety.
        //----------------------------------------------------------------------------------------------
        msStuckDetectInit = Settings.msStuckDetectInit;
        msStuckDetectInitLoop = Settings.msStuckDetectInitLoop;
        msStuckDetectStart = Settings.msStuckDetectStart;
        msStuckDetectLoop = Settings.msStuckDetectLoop;
        msStuckDetectStop = Settings.msStuckDetectStop;

        robot.hardwareMap = hardwareMap;
        robot.telemetry = telemetry;
        robot.init();
        telemetry.addData("Test AutonTopBParkAsc", "Initialized");

        //Initialize Gyro
        robot.driveTrain.ResetGyro();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        // initialize robot
        robot.init_loop();

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        // start robot
        runtime.reset();
        robot.start();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        telemetry.addData("Auton_Current_Stage ", currentStage);
        robot.loop();

        switch (currentStage){
            case  _unknown:
                currentStage = stage._00_preStart;
                break;
            case _00_preStart:
                currentStage = stage._10_Drive_Out;
                break;
            case _10_Drive_Out:
                robot.driveTrain.CmdDrive(1,0,0.35,0);
                currentStage = stage._20_Strafe_Left;
                break;
            case _20_Strafe_Left:
                if(robot.driveTrain.getCmdComplete()){
                    robot.driveTrain.cmdTurn(-90, 0.35);
                    currentStage = stage._25_Drive_Back;

                }
                break;
            case _25_Drive_Back:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.CmdDrive(10, -179, 0.35, -90);
                    currentStage = stage._28_Drive_To_Basket;
                }
                break;
            case _28_Drive_To_Basket:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.CmdDrive(11, -90, 0.35, -90);
                    currentStage = stage._29_Drive_To_Wall;
                }
                break;
            case _29_Drive_To_Wall:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.CmdDrive(4, -179, 0.35, -90);
                    currentStage = stage._30_Strafe_Right;
                }

                break;
            case _30_Strafe_Right:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.arm.setCurrentMode(Arm.Mode.DELIVER_TO_HIGH_BASKET_ARM_ONLY);
                    currentStage = stage._40_Drive_Forward;
                }
                break;
            case _40_Drive_Forward:
                if (robot.arm.getCmdComlete())  {
                    robot.intake.setCurrentMode(Intake.Mode.OUT);
                    runtime.reset();
                    currentStage = stage._50_Strafe_Right;
                }
                break;
            /*case _45_Back_Up:
                if(runtime.milliseconds() > 3000){
                    robot.driveTrain.CmdDrive(1,90,0.35,-90);
                    currentStage = stage._50_Strafe_Right;
                }
                break;*/
            case _50_Strafe_Right:
                if (runtime.milliseconds() > 2700) {
                    robot.intake.setCurrentMode(Intake.Mode.STOP);
                    robot.arm.setCurrentMode(Arm.Mode.RETRACT_TO_NEUTRAL_POS);
                    currentStage = stage._55_Turn;
                    runtime.reset();
                }
                break;
            case _55_Turn:
                if(runtime.milliseconds() > 3000){
                    robot.driveTrain.cmdTurn(0, 0.35);
                    currentStage = stage._70_Drive_Forward;
                }
                break;
            /*case _60_Turn:
                if (robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.CmdDrive(50, 0, 0.35, 0);
                    currentStage = stage._100_End;
                }

                break;*/
            case _70_Drive_Forward:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.CmdDrive(39, 0, 0.35, 0);
                    currentStage = stage._80_Strafe_Right;
                }

                break;
            case _80_Strafe_Right:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.CmdDrive(22, 90, 0.35,0 );
                    currentStage = stage._90_Arm_Start;
                }
                break;
            case _90_Arm_Start:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.arm.setCurrentMode(Arm.Mode.START);
                    currentStage = stage._100_End;
                }
                break;
            case _100_End:
                if(robot.driveTrain.getCmdComplete()){
                    robot.stop();


                }

                break;
        }



    }  //  loop

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.stop();
    }

    private enum stage {
        _unknown,
        _00_preStart,
        _10_Drive_Out,
        _20_Strafe_Left,
        _25_Drive_Back,
        _28_Drive_To_Basket,
        _29_Drive_To_Wall,
        _30_Strafe_Right,
        _40_Drive_Forward,
        _45_Back_Up,
        _50_Strafe_Right,
        _55_Turn,
        _60_Turn,
        _70_Drive_Forward,
        _80_Strafe_Right,
        _90_Arm_Start,
        _100_End


    }
}