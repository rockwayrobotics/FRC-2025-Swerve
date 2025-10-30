package frc.robot.util;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableListener;
import edu.wpi.first.wpilibj.Preferences;
import java.util.EnumSet;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

/**
 * A utility class for creating tunable number values that can be persisted
 *
 * <p>between robot restarts.
 *
 * <p>The values can be adjusted through NetworkTables and optionally saved in WPILib Preferences.
 */
public class Tuner {
  private final String key;
  private final double defaultValue;
  private final DoubleSubscriber subscriber;
  private final DoubleTopic topic;

  /** defaultValue will be ignored if persist is true, and has been previously set */
  public Tuner(String name, double defaultValue, boolean persist) {
    this.key = name;
    var nt = NetworkTableInstance.getDefault();
    topic = nt.getDoubleTopic(String.join("", "/Tuning/", name));

    if (persist && Preferences.containsKey(key)) {
      this.defaultValue = Preferences.getDouble(key, defaultValue);
    } else {
      this.defaultValue = defaultValue;
    }
    Logger.recordOutput("/Tuning/Defaults/" + name, this.defaultValue);

    this.subscriber = topic.subscribe(this.defaultValue);
    topic.publish().set(this.defaultValue);

    if (persist) {
      Preferences.setDouble(key, this.defaultValue);
      NetworkTableListener.createListener(
          this.subscriber,
          EnumSet.of(NetworkTableEvent.Kind.kValueAll),
          (event) -> {
            Preferences.setDouble(key, this.subscriber.get());
          });
    }
  }

  public double get() {
    return subscriber.get();
  }

  public void set(double value) {
    topic.publish().set(value);
  }

  public NetworkTableListener addListener(Consumer<NetworkTableEvent> callback) {
    return NetworkTableListener.createListener(
        this.subscriber, EnumSet.of(NetworkTableEvent.Kind.kValueAll), callback);
  }
}
