## SwerveModule.java

Controls the steering and driving for single swerve module.

Upgrades over 2023 implementation:
- Commanding velocity directly instead of crude voltage command.
    - The old implementation commanded voltage using `speed * (maxvoltage/maxspeed)`. This approach would fail if `maxspeed` was ever changed
    and didn't account for the actual behavior of the motors when loaded with the weight of the robot. We were also using an incorrect `maxspeed`.
    Essentially, the old approach only used FF and had no feedback loop, and the FF wasn't tuned properly.
- Using more straightforward implementation of SwerveModule.
    - The SDS SwerveModule was extremely convoluted and hard to understand. It used 3 levels of interfaces, factories, and factory builders. This is
    far too verbose for our purposes and the new implementation is easier to understand for future developers.

Notes:
- Steer Relative Encoder has units of module rotation in radians and radians/s
- Steer Absolute Encoder has units of module rotation in degrees
- Drive Relative Encoder has units of wheel travel (robot travel) in meters and meter/s
- Feedforward gain units are in Volts/Meters/Sec

Todo:
- [**DONE**] *PID/FF Gains*
- Motors inverted?
- [**DONE**] *Module steer offsets*
- Motors run at very slow speed when joystick is neutral.
- [**DONE**] *State optimization doesn't work ie. module will rotate >90 degrees*

Tests:
1. *Should the drive motor be controlled using a custom controller or the integrated velocity controller?* **Custom controller with FF and PID. FF gets the velocity to within 3% when running at 3m/s and PID can compensate for that error.**
    - *In other implementations the drive motor is controlled with voltage directly, not by velocity.*
    - *The WPILib example uses custom velocity PID + FF. SDS used voltage.*
2. *Should the steer controller be a profiled (trapezoidal) controller instead of the integrated spark max one?* **No, it was hard to tune and a normal controller worked better. Used**
    - The WPILib example uses a trapezoidal controller (+ FF). The SDS implementation uses the integrated controller.
    - WPILibControllerSwerveModule implements this.
3. *Is it worth it to measure FF values for everything?* **Yes, it was very easy. Just apply a constant voltage and read out velocity from encoder**
    - *We already have theoretical FF for unloaded motors.*
4. Should the relative encoder be periodically updated using the absolute encoder?
    - The SDS implementation updated the relative encoder periodically incase the reading taken from the absolute encoder on startup was bad.
5. *Does changing the encoder conversion factors change the units of the controller?* **Yes**
    - *Control units might not be right.* 
    - *Documentation seems to suggest this is true, should still be tested.*

___