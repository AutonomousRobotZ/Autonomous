import 'package:mqtt_client/mqtt_client.dart';
import 'package:mqtt_client/mqtt_browser_client.dart';

class MQTTService {
  late MqttBrowserClient client;
  Function(String topic, String message)? onMessageReceived;

  MQTTService() {
    
    client = MqttBrowserClient('ws://192.168.1.9:8083/mqtt', 'flutter_web_client_${DateTime.now().millisecondsSinceEpoch}');

    client.port = 8083;
    client.keepAlivePeriod = 20;
    client.logging(on: false);
    client.onDisconnected = _onDisconnected;
    client.onConnected = _onConnected;
    client.setProtocolV311(); // Important pour compatibilité

    final connMessage = MqttConnectMessage()
        .withClientIdentifier('flutter_web_client_${DateTime.now().millisecondsSinceEpoch}')
        .startClean()
        .withWillQos(MqttQos.atMostOnce);

    client.connectionMessage = connMessage;
  }

  Future<void> connect() async {
    try {
      await client.connect();
    } catch (e) {
      print("❌ Erreur de connexion MQTT : $e");
      client.disconnect();
    }

    client.updates?.listen((List<MqttReceivedMessage<MqttMessage>> events) {
      final topic = events[0].topic;
      final payload =
          (events[0].payload as MqttPublishMessage).payload.message;
      final message = MqttPublishPayload.bytesToStringAsString(payload);
      print('📥 Message reçu sur [$topic] : $message');
      onMessageReceived?.call(topic, message);
    });
  }
   void publish(String topic, String message) {
    final builder = MqttClientPayloadBuilder();
    builder.addString(message);
    client.publishMessage(topic, MqttQos.atLeastOnce, builder.payload!);
  }


  void _onConnected() {
    print("✅ Connecté au broker !");
    client.subscribe("robotPhoenix/sensors/flame", MqttQos.atMostOnce);
    client.subscribe("robotPhoenix/sensors/gps", MqttQos.atMostOnce);
  }

  void _onDisconnected() {
    print("⚠️ Déconnecté du broker");
  }

  void disconnect() {
    client.disconnect();
  }
}
