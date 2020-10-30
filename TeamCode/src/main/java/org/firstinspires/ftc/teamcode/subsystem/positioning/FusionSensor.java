package org.firstinspires.ftc.teamcode.subsystem.positioning;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.filter.KalmanFilter;
import org.apache.commons.math3.filter.MeasurementModel;
import org.apache.commons.math3.filter.ProcessModel;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

import java.util.function.Supplier;

public class FusionSensor extends PositioningSensor {

	KalmanFilter filter;

	@Override
	public Supplier<Position> getPositionSupplier() {
		return null;
	}

	@Override
	public void initialize(HardwareMap hardwareMap) {
		filter = new KalmanFilter(new ProcessModel() {
			@Override
			public RealMatrix getStateTransitionMatrix() {
				return null;
			}

			@Override
			public RealMatrix getControlMatrix() {
				return null;
			}

			@Override
			public RealMatrix getProcessNoise() {
				return null;
			}

			@Override
			public RealVector getInitialStateEstimate() {
				return null;
			}

			@Override
			public RealMatrix getInitialErrorCovariance() {
				return null;
			}
		}, new MeasurementModel() {
			@Override
			public RealMatrix getMeasurementMatrix() {
				return null;
			}

			@Override
			public RealMatrix getMeasurementNoise() {
				return null;
			}
		});
		priority = 10;
	}

	@Override
	public void initPeriodic() {

	}

	@Override
	public void start() {

	}

	@Override
	public void runPeriodic() {

	}

	@Override
	public void stop() {

	}
}
