package frc.robot.Laptop;

import java.io.FileWriter;
import java.io.IOException;
import java.util.EnumSet;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableListener;
import edu.wpi.first.networktables.StringSubscriber;

/*
Only class without every method robo has? Basically only for file writing so doesn't make sense to. I think so anyways
 */
public class FileUpdater {
    private NetworkTableInstance myInstance;
    //Publishes true if the laptop is open to accepting a file, usually only false if it's currently writing
    private BooleanPublisher accepting;
    //Gets the path of the file to write to
    private StringSubscriber path;
    //Gets the content to write into the file
    private StringSubscriber content;
    //A listener to listen to events in the network folder (all the above are files in the folder)
    //The folder is /laptop/fileUpdater/
    @SuppressWarnings("unused")
    private NetworkTableListener fileUpdateListener;

    public FileUpdater(NetworkTableInstance inst) {
        myInstance = inst;
    }

    public void create() {
        accepting = myInstance.getBooleanTopic("/laptop/fileUpdater/accepting").publish();
        path = myInstance.getStringTopic("/laptop/fileUpdater/path").subscribe("");
        content = myInstance.getStringTopic("/laptop/fileUpdater/content").subscribe("");

        fileUpdateListener = NetworkTableListener.createListener(myInstance.getTopic("/laptop/fileUpdater"), EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> {
            //Despite the name kValueAll, it is only triggered when topic values are update (so either path or content is updated)
            //You might be saying, wait, shouldn't I check if path AND content changed
            //Generally, both path and content can be changed before the table is flushed
            //If by chance that doesn't happen, then content almost certainly will be update to date by the time we get around to writing
            if(event.is(NetworkTableEvent.Kind.kValueAll)) {
                //No longer accepting
                accepting.set(false);
                //Push changes to table
                myInstance.flush();

                try {
                    //Don't append---replace whatever is in the file
                    FileWriter author = new FileWriter(path.get(), false);
                    author.write(content.get(), 0, content.get().length());
                    author.close();
                } catch(IOException e) {
                    e.printStackTrace();
                } finally {
                    //Accepting again
                    accepting.set(true);
                    myInstance.flush();
                }
            }
        }); //end Listener construction

        accepting.set(true);
        myInstance.flush();
    }

    public void close() {
        accepting.close();
        accepting = null;
        path.close();
        path = null;
        content.close();
        content = null;
        fileUpdateListener.close();
        fileUpdateListener = null;
    }
}
