// main.dart

import 'package:flutter/material.dart';
import 'package:flutter/foundation.dart';
import 'mqtt_service.dart';
import 'package:flutter_map/flutter_map.dart';
import 'package:latlong2/latlong.dart';
import 'dart:html' as html;
import 'dart:ui' as ui;

void main() {
  runApp(const PhoenixApp());
}

class PhoenixApp extends StatelessWidget {
  const PhoenixApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Phoenix Robot',
      debugShowCheckedModeBanner: false,
      theme: ThemeData.dark().copyWith(
        scaffoldBackgroundColor: const Color(0xFF121212),
        appBarTheme: const AppBarTheme(
          backgroundColor: Color(0xFFFA541C),
          elevation: 3,
          titleTextStyle: TextStyle(fontSize: 20, fontWeight: FontWeight.bold),
        ),
        elevatedButtonTheme: ElevatedButtonThemeData(
          style: ElevatedButton.styleFrom(
            backgroundColor: const Color(0xFFFA541C),
            foregroundColor: Colors.white,
            shape: RoundedRectangleBorder(
              borderRadius: BorderRadius.circular(12),
            ),
            padding: const EdgeInsets.symmetric(vertical: 14, horizontal: 28),
            textStyle: const TextStyle(fontSize: 18, fontWeight: FontWeight.w600),
          ),
        ),
        bottomNavigationBarTheme: const BottomNavigationBarThemeData(
          backgroundColor: Colors.black,
          selectedItemColor: Color(0xFFFA541C),
          unselectedItemColor: Colors.grey,
        ),
      ),
      home: const PhoenixHome(),
    );
  }
}

class PhoenixHome extends StatefulWidget {
  final int initialIndex;
  const PhoenixHome({super.key, this.initialIndex = 0});

  @override
  State<PhoenixHome> createState() => _PhoenixHomeState();
}

class _PhoenixHomeState extends State<PhoenixHome> {
  late int _currentIndex;

  @override
  void initState() {
    super.initState();
    _currentIndex = widget.initialIndex;
  }

  final List<Widget> _pages = [
    const HomePage(),
    const CameraStreamPage(),
    const AutoModePage(),
    const HistoryPage(),
  ];

  void changePage(int index) {
    setState(() => _currentIndex = index);
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: IndexedStack(index: _currentIndex, children: _pages),
      bottomNavigationBar: BottomNavigationBar(
        currentIndex: _currentIndex,
        onTap: changePage,
        items: const [
          BottomNavigationBarItem(icon: Icon(Icons.home), label: 'Accueil'),
          BottomNavigationBarItem(icon: Icon(Icons.camera_alt), label: 'Caméra'),
          BottomNavigationBarItem(icon: Icon(Icons.android), label: 'Auto'),
          BottomNavigationBarItem(icon: Icon(Icons.map), label: 'GPS'),
        ],
      ),
    );
  }
}

class HomePage extends StatelessWidget {
  const HomePage({super.key});

  @override
  Widget build(BuildContext context) {
    final isMobile = MediaQuery.of(context).size.width < 600;

    return Scaffold(
      body: Center(
        child: SingleChildScrollView(
          padding: const EdgeInsets.symmetric(horizontal: 24),
          child: Column(
            mainAxisAlignment: MainAxisAlignment.center,
            children: [
              Container(
  padding: const EdgeInsets.all(24),
  decoration: BoxDecoration(
    gradient: const LinearGradient(
      colors: [Color.fromARGB(255, 0, 0, 0), Color.fromARGB(255, 0, 0, 0)], // Plus clair que 1C1C1C
      begin: Alignment.topLeft,
      end: Alignment.bottomRight,
    ),
    borderRadius: BorderRadius.circular(20),
    boxShadow: [
      BoxShadow(
        color: Colors.deepOrangeAccent.withOpacity(0.4),
        blurRadius: 20,
        offset: const Offset(0, 10),
      ),
    ],
  ),
  child: Image.asset(
    'assets/logo.jpg',
    height: isMobile ? 240 : 280,
  ),
),


              const SizedBox(height: 48),
              _navButton(context, Icons.camera_alt, "Caméra", 1),
              const SizedBox(height: 20),
              _navButton(context, Icons.android, "Mode Auto", 2),
              const SizedBox(height: 20),
              _navButton(context, Icons.map, "GPS", 3),
            ],
          ),
        ),
      ),
    );
  }

  Widget _navButton(BuildContext context, IconData icon, String label, int index) {
    return ElevatedButton.icon(
      onPressed: () {
        Navigator.of(context).pushReplacement(
          MaterialPageRoute(builder: (_) => PhoenixHome(initialIndex: index)),
        );
      },
      icon: Icon(icon, size: 24),
      label: Text(label),
    );
  }
}

class CameraStreamPage extends StatelessWidget {
  const CameraStreamPage({super.key});
  final String streamUrl = 'http://192.168.1.9:8081';

  @override
  Widget build(BuildContext context) {
    if (kIsWeb) {
      const viewType = 'mjpeg-stream';
      final html.ImageElement imageElement = html.ImageElement()
        ..src = streamUrl
        ..style.width = '100%'
        ..style.height = '100%'
        ..style.objectFit = 'cover';

      // ignore: undefined_prefixed_name
      ui.platformViewRegistry.registerViewFactory(viewType, (int viewId) => imageElement);

      return Scaffold(
        appBar: AppBar(title: const Text('Flux Caméra')),
        body: Center(
          child: AspectRatio(
            aspectRatio: 4 / 3,
            child: HtmlElementView(viewType: viewType),
          ),
        ),
      );
    } else {
      return const Scaffold(
        body: Center(child: Text('Non supporté ici')),
      );
    }
  }
}

class AutoModePage extends StatefulWidget {
  const AutoModePage({super.key});

  @override
  State<AutoModePage> createState() => _AutoModePageState();
}

class _AutoModePageState extends State<AutoModePage> {
  final MQTTService mqttService = MQTTService();
  bool _autoModeActivated = false;

  @override
  void initState() {
    super.initState();
    mqttService.connect();
  }

  @override
  void dispose() {
    mqttService.disconnect();
    super.dispose();
  }

  void _toggleAutoMode() {
    setState(() {
      _autoModeActivated = !_autoModeActivated;
    });

    mqttService.publish('robot/control', _autoModeActivated ? 'auto' : 'stop');

    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(
            _autoModeActivated ? 'Mode Auto Activé' : 'Mode Auto Désactivé'),
        backgroundColor: Colors.deepOrange,
        duration: const Duration(seconds: 2),
      ),
    );
  }

  void _sendDirection(String direction) {
    mqttService.publish('robot/control', direction);
  }

  Widget _directionButton(IconData icon, String direction) {
    return ElevatedButton(
      onPressed: () => _sendDirection(direction),
      style: ElevatedButton.styleFrom(
        shape: const CircleBorder(),
        backgroundColor: Colors.deepOrange,
        padding: const EdgeInsets.all(22),
        elevation: 6,
        shadowColor: Colors.deepOrangeAccent,
      ),
      child: Icon(icon, size: 36, color: Colors.white),
    );
  }

  @override
  Widget build(BuildContext context) {
    final isMobile = MediaQuery.of(context).size.width < 600;

    return Scaffold(
      appBar: AppBar(title: const Text('Mode Automatique')),
      body: Stack(
        children: [
          // 🧱 Arrière-plan avec logo couvrant toute la page
          Positioned.fill(
            child: Container(
  decoration: const BoxDecoration(
    gradient: LinearGradient(
      begin: Alignment.topLeft,
      end: Alignment.bottomRight,
      colors: [
        Color.fromARGB(255, 18, 18, 18), // Fond sombre
        Color.fromARGB(255, 64, 64, 64), // Variation
      ],
    ),
  ),
  child: Opacity(
    opacity: 0.08, // Le logo est en fond, très atténué
    child: Center(
     child: Container(
  decoration: BoxDecoration(
    boxShadow: [
      BoxShadow(
        color: Colors.black.withOpacity(0.3),
        blurRadius: 12,
        offset: const Offset(0, 6),
      ),
    ],
  ),
  child: Image.asset(
    'assets/logo.jpg',
    height: isMobile ? 480 : 520,
  ),
),

    ),
  ),
),

          ),

          // 🧭 Contrôles
          Center(
            child: SingleChildScrollView(
              padding: const EdgeInsets.symmetric(horizontal: 24, vertical: 16),
              child: Column(
                mainAxisSize: MainAxisSize.min,
                children: [
                  ElevatedButton.icon(
                    onPressed: _toggleAutoMode,
                    icon: Icon(
                        _autoModeActivated ? Icons.close : Icons.play_arrow),
                    label: Text(_autoModeActivated
                        ? 'Désactiver Mode Auto'
                        : 'Activer Mode Auto'),
                    style: ElevatedButton.styleFrom(
                      backgroundColor: _autoModeActivated
                          ? Colors.redAccent
                          : Colors.deepOrange,
                      foregroundColor: Colors.white,
                      padding: const EdgeInsets.symmetric(
                          horizontal: 30, vertical: 16),
                      textStyle: const TextStyle(
                        fontSize: 18,
                        fontWeight: FontWeight.bold,
                      ),
                      shape: RoundedRectangleBorder(
                        borderRadius: BorderRadius.circular(14),
                      ),
                      elevation: 8,
                    ),
                  ),
                  const SizedBox(height: 40),

                  // 🎯 Direction arrows
                  AnimatedOpacity(
                    duration: const Duration(milliseconds: 400),
                    opacity: _autoModeActivated ? 1 : 0,
                    child: IgnorePointer(
                      ignoring: !_autoModeActivated,
                      child: Column(
                        children: [
                          _directionButton(Icons.arrow_upward, 'haut'),
                          const SizedBox(height: 20),
                          Row(
                            mainAxisAlignment: MainAxisAlignment.center,
                            children: [
                              _directionButton(Icons.arrow_back, 'gauche'),
                              const SizedBox(width: 60),
                              _directionButton(Icons.arrow_forward, 'droite'),
                            ],
                          ),
                          const SizedBox(height: 20),
                          _directionButton(Icons.arrow_downward, 'bas'),
                        ],
                      ),
                    ),
                  ),
                ],
              ),
            ),
          ),
        ],
      ),
    );
  }
}


class HistoryPage extends StatefulWidget {
  const HistoryPage({super.key});

  @override
  State<HistoryPage> createState() => _HistoryPageState();
}


class _HistoryPageState extends State<HistoryPage> {
  final MQTTService mqttService = MQTTService();
final LatLng ensiCoords = const LatLng(36.813556, 10.063911);
  late LatLng _currentPosition;
  late MapController _mapController;

  @override
  void initState() {
    super.initState();
    _currentPosition = ensiCoords;
    _mapController = MapController();

    mqttService.onMessageReceived = (topic, message) {
      final parts = message.split(',');
      if (parts.length == 2) {
        final lat = double.tryParse(parts[0]);
        final lon = double.tryParse(parts[1]);
        if (lat != null && lon != null) {
          setState(() {
            _currentPosition = LatLng(lat, lon);
            _mapController.move(_currentPosition, _mapController.camera.zoom);
          });
        }
      }
    };

    mqttService.connect();
  }

  @override
  void dispose() {
    mqttService.disconnect();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('Historique GPS')),
      body: FlutterMap(
        mapController: _mapController,
        options: MapOptions(
          initialCenter: ensiCoords,
          initialZoom: 16,
        ),
        children: [
          TileLayer(
            urlTemplate: 'https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',
            subdomains: const ['a', 'b', 'c'],
          ),
          MarkerLayer(
            markers: [
              Marker(
                point: _currentPosition,
                width: 40,
                height: 40,
                child: const Icon(
                  Icons.location_on,
                  size: 40,
                  color: Colors.deepOrangeAccent,
                ),
              ),
            ],
          ),
        ],
      ),
      floatingActionButton: FloatingActionButton(
        backgroundColor: Colors.deepOrange,
        onPressed: () => _mapController.move(ensiCoords, 16),
        child: const Icon(Icons.gps_fixed),
      ),
    );
  }
}