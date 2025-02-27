package frc.robot.util;

import java.util.EnumSet;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;

public class LimelightMirror {
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable cameraTable = inst.getTable("CameraPublisher");

    private final StringPublisher description;
    private final StringPublisher mode;
    private final StringArrayPublisher modes;
    private final StringPublisher source;
    private final StringArrayPublisher streams;

    public LimelightMirror(String srcName, String dstName, int streamPort) {
        final String pfx = dstName + "/";

        description = cameraTable.getStringTopic(pfx + "description").publish();
        mode = cameraTable.getStringTopic(pfx + "mode").publish();
        modes = cameraTable.getStringArrayTopic(pfx + "modes").publish();
        source = cameraTable.getStringTopic(pfx + "source").publish();
        streams = cameraTable.getStringArrayTopic(pfx + "streams").publish();

        cameraTable.addListener(
            srcName,
            EnumSet.of(Kind.kPublish, Kind.kValueAll),
            new NetworkTable.TableEventListener() {
                @Override
                public void accept(NetworkTable table, String key, NetworkTableEvent event) {
                    switch (key) {
                        case "mode":
                            mode.set(event.valueData.value.getString());
                            break;
                        case "modes":
                            modes.set(event.valueData.value.getStringArray());
                            break;
                        /*case "source":
                            source.set(event.valueData.value.getString());
                            break;
                        case "streams":
                            streams.set(event.valueData.value.getStringArray());
                            break;
                        default:
                            System.out.println("Got unexpected update: "+key);
                            break;*/
                    }
                }
            }
        );

        final String sp = srcName + "/";
        description.set(dstName);
        try {
            mode.set(cameraTable.getValue(sp+"mode").getString());
            modes.set(cameraTable.getValue(sp+"modes").getStringArray());
        } catch (ClassCastException ignored) {} // in sim mode, we don't have these available
        source.set("ip:http://10.16.61.2:"+streamPort);
        streams.set(new String[] {"mjpg:http://10.16.61.2:"+streamPort});
    }
}
