package org.eclipse.paho.client.mqttv3;
/** API micropaho (MicroEJ): hanya connectionLost & messageArrived. deliveryComplete dibuat default. */
public interface MqttCallback {
    void connectionLost(Throwable cause);
    void messageArrived(String topic, MqttMessage message) throws Exception;
    default void deliveryComplete(IMqttDeliveryToken token) { }
}
