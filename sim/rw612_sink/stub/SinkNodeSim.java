import com.nxp.example.smartgreenhouse.controllers.ActuatorDetailController;
import com.nxp.example.smartgreenhouse.models.actuator.ActuatorDataStore;
import com.nxp.example.smartgreenhouse.services.mqtt.MqttSubscribeService;
/** Menjalankan MqttSubscribeService asli RW612 seperti AppController di board (WiFi sudah terhubung). */
public final class SinkNodeSim {
    public static void main(String[] args) throws Exception {
        System.setProperty("java.util.logging.SimpleFormatter.format", "%1$tT.%1$tL %4$s %3$s %5$s%n");
        MqttSubscribeService svc = new MqttSubscribeService(new ActuatorDataStore(), new ActuatorDetailController());
        svc.start();
        System.out.println("SINK_NODE_SIM_STARTED broker=" + System.getProperty("sim.broker"));
        Thread.currentThread().join();
    }
}
