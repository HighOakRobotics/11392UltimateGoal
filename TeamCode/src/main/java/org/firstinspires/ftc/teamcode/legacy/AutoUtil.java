package org.firstinspires.ftc.teamcode.legacy;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.legacy.Holonomic;

import java.util.concurrent.TimeUnit;

//import org.firstinspires.ftc.teamcode.opmodes.prod.Drive;
//import org.ftc11392.legacy.subsystems.IMU;

//Everything in this class in synchronous.
public class AutoUtil {
	/**
	 * Turns left however many degrees using the Holonomic class's dstDrive. Cannot turn more than 360 degrees.
	 * Note that this method is synchronous - it finishes the turn before exiting the method call. (However, the
	 * method does stop in the case of the opMode stopping thanks to the this parameter.)
	 * @param degrees The number of degrees you would like to turn left.
	 *                PRECONDITION: 0 < degrees < 360.
	 * @param holy The holonomic object that you would like to move.
	 * @param imu The IMU object attached to obtain angular orientation with.
	 * @param justPutThis Just put the keyword "this" in the parameter field. It was the best way we
	 *                    figured out to make isStopRequested() and opModeIsActive() functional.
	 */
	public static void turnLeft(double degrees, Holonomic holy, BNO055IMU imu, LinearOpMode justPutThis){

		/* First, we need to convert from the IMU's weird angle system to a 0-360 degree angle system.
		 * The default IMU angle system is -180 at the bottom, up to 0 at the top through the right SIDE, and going down the left goes back to 180.
		 * We're adding 180 to transform this angle to somewhere between 0 and 360 degrees, starting at the bottom and rotating counterclockwise.
		 * Since we're turning left (AKA counterclockwise), we are going to add to our degrees.
		 */

		//Get the current angle and initialize the target angle as the sum of the current angle and the degrees in the turn
		Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
		double currentAngle = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle)) + 180;
		double starterAngle = currentAngle;
		double targetAngle = currentAngle + degrees;

		boolean overloaded = targetAngle > 360; //If the targetAngle has to loop back around, overloaded becomes true.
		targetAngle = (targetAngle > 360) ? (targetAngle - 360) : targetAngle;

		boolean targetPassed = false;

		//Start driving. (Turn goes at full power)
		holy.dstDrive(0,0,-0.5);

		//While the opmode is running, there is no stop, and the target is not passed, we continue running the loop.
		//The first two terms only exist to stop on stops.
		while(justPutThis.opModeIsActive() && !justPutThis.isStopRequested() && !targetPassed){
			angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
			currentAngle = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle)) + 180;

			//If the currentAngle has yet to loop back around (by passing the 180 IMU / 360 our system mark), overloaded remains true.
			//If overload remains true, we do a check to see if currentAngle has looped back around.
			if(overloaded){
				overloaded = currentAngle > starterAngle;
			}
			/* Otherwise...
			 * If overloaded is false, we know that either nothing has ever looped around or both of the variables currentAngle and targetAngle
			 * have looped. This means that at this point, currentAngle < targetAngle, and all we need to do is wait for currentAngle to
			 * become greater than targetAngle. While the angle is still overloaded, the targetAngle cannot have been passed, thus this check
			 * is only run if the angles are no longer overloaded. If it is true, we set targetPassed to true and kick out of the while loop.
			 */
			else {
				targetPassed = currentAngle > targetAngle;
			}
		}

		//Once the while loop has kicked out, we know that the target has been passed.
		//Thus, we are safe to stop the robot's movement.
		holy.stop();
	}

	/**
	 * Turns right however many degrees using the Holonomic class's dstDrive. Cannot turn more than 360 degrees.
	 * Note that this method is synchronous - it finishes the turn before exiting the method call. (However, the
	 * method does stop in the case of the opMode stopping thanks to the this parameter.)
	 * @param degrees The number of degrees you would like to turn right.
	 *                PRECONDITION: 0 < degrees < 360.
	 * @param holy The holonomic object that you would like to move.
	 * @param imu The IMU object attached to obtain angular orientation with.
	 * @param justPutThis Just put the keyword "this" in the parameter field. It was the best way we
	 *                    figured out to make isStopRequested() and opModeIsActive() functional.
	 */
	public static void turnRight(double degrees, Holonomic holy, BNO055IMU imu, LinearOpMode justPutThis){

		/* First, we need to convert from the IMU's weird angle system to a 0-360 degree angle system.
		 * The default IMU angle system is -180 at the bottom, up to 0 at the top through the right SIDE, and going down the left goes back to 180.
		 * We're adding 180 to transform this angle to somewhere between 0 and 360 degrees, starting at the bottom and rotating counterclockwise.
		 * Since we're turning right (AKA clockwise), we are going to subtract from our degrees.
		 */

		//Get the current angle and initialize the target angle as the difference between the current angle and the degrees in the turn
		Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
		double currentAngle = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle)) + 180;
		double starterAngle = currentAngle;
		double targetAngle = currentAngle - degrees;

		boolean overloaded = targetAngle < 0; //If the targetAngle has to loop back around, overloaded becomes true.
		targetAngle = (targetAngle < 0) ? (targetAngle + 360) : targetAngle; //Loop targetAngle back around if it's overloaded

		boolean targetPassed = false;

		//Start driving. (Turn goes at full power)
		holy.dstDrive(0,0,0.5);

		//While the opmode is running, there is no stop, and the target is not passed, we continue running the loop.
		//The first two terms only exist to stop on stops.
		while(justPutThis.opModeIsActive() && !justPutThis.isStopRequested() && !targetPassed){
			angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
			currentAngle = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle)) + 180;

			//If the currentAngle has yet to loop back around (by passing the -180 IMU / 0 our system mark), overloaded remains true.
			//If overload remains true, we do a check to see if currentAngle has looped back around.
			if(overloaded){
				overloaded = currentAngle < starterAngle;
			}
			/* Otherwise...
			 * If overloaded is false, we know that either nothing has ever looped around or both of the variables currentAngle and targetAngle
			 * have looped. This means that at this point, currentAngle < targetAngle, and all we need to do is wait for currentAngle to
			 * become greater than targetAngle. While the angle is still overloaded, the targetAngle cannot have been passed, thus this check
			 * is only run if the angles are no longer overloaded. If it is true, we set targetPassed to true and kick out of the while loop.
			 */
			else {
				targetPassed = currentAngle < targetAngle;
			}
		}

		//Once the while loop has kicked out, we know that the target has been passed.
		//Thus, we are safe to stop the robot's movement.
		holy.stop();
	}

	public static void imuPIDTurnAbsoluteSynch(double degrees, double power, BNO055IMU imu, Holonomic holy, LinearOpMode justPutThis, Telemetry telemetry, double Kp, double Ki, double Kd, double iMax, double margin){
		holy.stopTargeting();

		Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
		double currDeg = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));

		ElapsedTime thyme = new ElapsedTime();

		long curTime;
		long recTime = -1;
		long prevTime = thyme.time(TimeUnit.MILLISECONDS);
		boolean recording = false;

		double p, i = 0, d;
		double diff = degrees - currDeg;
		if(diff > 180){
			diff = -(360 - diff);
		}
		if(diff < -180){
			diff = -(-360 - diff);
		}
		double prevDiff = diff;

		while(justPutThis.opModeIsActive() && !justPutThis.isStopRequested()){
			//update current degree
			angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
			currDeg = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));

			curTime = thyme.time(TimeUnit.MILLISECONDS);

			diff = degrees - currDeg;

			if(diff > 180){
				diff = -(360 - diff);
			}
			if(diff < -180){
				diff = -(-360 - diff);
			}

			p = Kp * diff;
			i += Ki * (diff * (curTime - prevTime));

			if(i > iMax)
				i = iMax;
			if(i < -iMax)
				i = -iMax;

			d = Kd * (diff - prevDiff) / (curTime - prevTime);

			holy.dstDrive(0, 0, Range.clip(-power * (p + i + d), -1.0, 1.0));

			prevTime = curTime;
			prevDiff = diff;

			if(Math.abs(diff) < margin){
				if (!recording) {
					recTime = thyme.time(TimeUnit.MILLISECONDS);
				}
				recording = true;
			}
			else {
				recording = false;
			}
			if(recording && (curTime - recTime) > 100){
				break;
			}

			telemetry.addData("diff", diff);
			telemetry.addData("lTime", curTime-prevTime);
			telemetry.addData("rTime", curTime-recTime);
			telemetry.addData("tTime", curTime);
			telemetry.addData("P,I,D", "%5.2f %5.2f %5.2f", p,i,d);
			telemetry.addData("PID", (p + i + d));
			telemetry.addData("recording", recording);
			telemetry.update();
		}
		holy.stop();
	}

	public static void imuTurnAbsoluteSynch (double degree, double speed, double margin, Holonomic holy, BNO055IMU imu, LinearOpMode justPutThis, Telemetry telemetry){
		Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
		double currDeg = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
		while (((Math.abs(degree - currDeg) > 180) ? (360 - Math.abs(degree - currDeg)) : (Math.abs(degree - currDeg))) > margin && justPutThis.opModeIsActive() && !justPutThis.isStopRequested()) {
			telemetry.addData("imudiff ", "%2.5f", ((Math.abs(degree - currDeg) > 180) ? (360 - Math.abs(degree - currDeg)) : (Math.abs(degree - currDeg))));

			if ((degree - currDeg) > 1) {
				holy.dstDrive(0.0, 0.0, -speed);
			}
			else if ((degree - currDeg) < -1) {
				holy.dstDrive(0.0, 0.0, speed);
			}

			telemetry.update();
			angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
			currDeg = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
		}
		telemetry.update();
		holy.stop();
	}

	public static void imu2StepTurnAbsoluteSynch(double degree, double speed1, double speed2, double margin1, double margin2, Holonomic holy, BNO055IMU imu, LinearOpMode justPutThis, Telemetry telemetry){
		imuTurnAbsoluteSynch(degree, speed1, margin1, holy, imu, justPutThis, telemetry);
		imuTurnAbsoluteSynch(degree, speed2, margin2, holy, imu, justPutThis, telemetry);
	}

	@Deprecated
	public static void turnToAbsoluteZero(double speed, Holonomic holy, BNO055IMU imu, LinearOpMode justPutThis){
		turnAbsolute2Step(0,speed,holy,imu,justPutThis);
	}

	@Deprecated
	public static void turnAbsolute2Step(int degree, double speed, Holonomic holy, BNO055IMU imu, LinearOpMode justPutThis) {
		turnToAbsolutePosition(degree, speed, holy, imu, justPutThis);
		turnToAbsolutePosition(degree, speed/2, holy, imu, justPutThis);
	}

	@Deprecated
	public static void imuTurn(BNO055IMU imu, double speed, double degree, double margin, Holonomic holy, Telemetry telemetry) {
		Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
		double currDeg = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
		while (Math.abs(degree - currDeg) > margin) {
			telemetry.addData("imudiff ", "%2.5f", degree - currDeg);
			if ((degree - currDeg) > 5.0) {
				holy.dstDrive(0.0, 0.0, -speed);
			}
			if ((degree - currDeg) < -5.0) {
				holy.dstDrive(0.0, 0.0, speed);
			}
			telemetry.update();
			angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
			currDeg = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
		}
		telemetry.update();
		holy.stop();
	}

	@Deprecated
	public static void imu2StepTurn(BNO055IMU imu, double speed1, double speed2, double degree, Holonomic holy, Telemetry telemetry ) {
		imuTurn(imu, speed1, degree, 1,holy, telemetry);
		imuTurn(imu, speed2, degree, 1,holy, telemetry);
	}

	@Deprecated
	public static void turnToAbsolutePosition(int degree, double speed, Holonomic holy, BNO055IMU imu, LinearOpMode justPutThis){
		Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
		double currentAngle = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle)) + 180;
		if (currentAngle < degree) {
			holy.dstDrive(0, 0, -speed);
			while (currentAngle < degree && justPutThis.opModeIsActive() && !justPutThis.isStopRequested()) { }
		} else {
			holy.dstDrive(0, 0, speed);
			while (currentAngle > degree && justPutThis.opModeIsActive() && !justPutThis.isStopRequested()) { }
		}

		holy.stop();
	}
}
