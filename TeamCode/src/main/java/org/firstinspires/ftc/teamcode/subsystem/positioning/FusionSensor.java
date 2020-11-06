package org.firstinspires.ftc.teamcode.subsystem.positioning;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.filter.KalmanFilter;
import org.apache.commons.math3.filter.MeasurementModel;
import org.apache.commons.math3.filter.ProcessModel;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

import java.util.function.Supplier;

public class FusionSensor extends PositioningSensor {

	KalmanFilter filter;
	Double output;

	@Override
	public Supplier<Position> getPositionSupplier() {
		return null;
	}

	@Override
	public void initialize(HardwareMap hardwareMap) {
		filter = new KalmanFilter(new ProcessModel() {
			@Override
			public RealMatrix getStateTransitionMatrix() {
				return MatrixUtils.createRealMatrix(new double[][]{
						{1.0, 0.0, 0.0},
						{0.0, 1.0, 0.0},
						{0.0, 0.0, 1.0}
				});
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
		output = 0.0;
		priority = 10;
	}

	public void update() {
		filter.predict();
		filter.correct(MatrixUtils.createRealVector(new double[]{}));
		output = filter.getStateEstimation()[0];
	}

	@Override
	public void initPeriodic() {
		update();
	}

	@Override
	public void runPeriodic() {
		update();
	}
}
