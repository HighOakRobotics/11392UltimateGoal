```java
public abstract class PositioningSensor extends Subsystem { ... } /* provides Position */
public class OdometrySensor extends PositioningSensor { ... }
public class VuforiaSensor extends PositioningSensor { ... }
public class IMUSensor extends PositioningSensor { ... }
public class FusionSensor extends PositioningSensor { ... } /* Sensor Fusion of the above positioning sensors */
public class FusionLocalizer implements Localizer { ... } /* conversion of above sensor fusion into a localizer road-runner understands */
```
as such...
 - `OdometrySensor` wraps `StandardTrackingWheelLocalizer` which extends `ThreeTrackingWheelLocalizer`
 - `VuforiaSensor` is an implementation of what is shown in `ConceptVuforiaNavigationWebcam`, 
 perhaps minus the OpenGL rendering and preview for performance. Data from `VuforiaTrackable` is translated into `Position`
 - `IMUSensor` should be a trivial implementation... IMU probably shouldn't provide x or y data since extrapolating position from acceleration is... not very good
 - `FusionSensor` is a `PositioningSensor` that attempts sensor fusion. We are planning to do this with 3 kalman filters. This may change.
 - `FusionLocalizer` translates data from `FusionSensor` into a format the `road-runner` drivetrain can understand
 
overall this system is major levels of cursed so we'll just be using `StandardTrackingWheelLocalizer` and `OdometrySensor` for the time being
 
**This architecture could have a lot of latency so it is subject to change.**