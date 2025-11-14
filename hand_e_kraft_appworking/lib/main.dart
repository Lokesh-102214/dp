import 'dart:async';
import 'dart:typed_data';
import 'package:flutter/material.dart';
import 'package:flutter_bluetooth_serial/flutter_bluetooth_serial.dart';
import 'package:fl_chart/fl_chart.dart';
import 'package:permission_handler/permission_handler.dart';
import 'package:shared_preferences/shared_preferences.dart';

void main() {
  runApp(const HandEKraftApp());
}

class HandEKraftApp extends StatelessWidget {
  const HandEKraftApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Hand-e-Kraft Controller',
      theme: ThemeData.dark().copyWith(
        scaffoldBackgroundColor: const Color(0xFF121212),
      ),
      home: const ControllerPage(),
      debugShowCheckedModeBanner: false,
    );
  }
}

class ControllerPage extends StatefulWidget {
  const ControllerPage({super.key});

  @override
  State<ControllerPage> createState() => _ControllerPageState();
}

class _ControllerPageState extends State<ControllerPage> {
  BluetoothConnection? connection;
  BluetoothDevice? targetDevice;
  bool isConnected = false;
  bool isConnecting = false;

  double tremorFreq = 0.0;
  double tremorPower = 0.0;
  double threshold = 0.0;

  String mode = "--";          // AUTO / MANUAL
  String servoStatus = "--";   // ENGAGED / RELAXED

  List<double> tremorFreqList = List.filled(50, 0);
  List<double> tremorPowerList = List.filled(50, 0);

  List<int> servoAngles = [90, 90, 90, 90, 90];

  String espStatus = "Searching for ESP32...";

  @override
  void initState() {
    super.initState();
    _initBluetooth();
  }

  Future<void> _initBluetooth() async {
    await Permission.bluetooth.request();
    await Permission.bluetoothConnect.request();
    await Permission.bluetoothScan.request();
    _autoConnectToSavedOrAvailable();
  }

  Future<void> _autoConnectToSavedOrAvailable() async {
    setState(() {
      isConnecting = true;
      espStatus = "Searching for ESP32...";
    });

    final prefs = await SharedPreferences.getInstance();
    String? savedAddress = prefs.getString("last_esp32_address");

    List<BluetoothDevice> devices =
        await FlutterBluetoothSerial.instance.getBondedDevices();

    if (savedAddress != null) {
      targetDevice = devices.firstWhere(
        (d) => d.address == savedAddress,
        orElse: () => devices.firstWhere(
            (d) => (d.name ?? "").contains("HandEKraft"),
            orElse: () => devices.isNotEmpty ? devices.first : devices.last),
      );
    } else {
      targetDevice = devices.firstWhere(
        (d) => (d.name ?? "").contains("HandEKraft"),
        orElse: () => devices.isNotEmpty ? devices.first : devices.last,
      );
    }

    if (targetDevice == null) {
      setState(() {
        isConnecting = false;
        espStatus = "No ESP32 found";
      });
      return;
    }

    await prefs.setString("last_esp32_address", targetDevice!.address);
    _connectToDevice(targetDevice!);
  }

  Future<void> _connectToDevice(BluetoothDevice device) async {
    setState(() => espStatus = "Connecting to ${device.name}...");

    try {
      BluetoothConnection con =
          await BluetoothConnection.toAddress(device.address);

      connection = con;
      setState(() {
        isConnected = true;
        isConnecting = false;
        espStatus = "Connected to ${device.name}";
      });

      connection!.input!.listen(_onDataReceived).onDone(() {
        setState(() {
          isConnected = false;
          espStatus = "Disconnected — retrying...";
        });
        Future.delayed(const Duration(seconds: 2), _autoConnectToSavedOrAvailable);
      });
    } catch (_) {
      setState(() {
        isConnected = false;
        isConnecting = false;
        espStatus = "Failed — retrying...";
      });
      await Future.delayed(const Duration(seconds: 4));
      _autoConnectToSavedOrAvailable();
    }
  }

  void _onDataReceived(Uint8List data) {
    String line = String.fromCharCodes(data).trim();

    if (!line.contains(":") && !line.contains(",")) return;

    String normalized = line.replaceAll("=", ":");

    for (String part in normalized.split(",")) {
      if (!part.contains(":")) continue;
      var kv = part.split(":");
      if (kv.length < 2) continue;

      String key = kv[0].trim().toUpperCase();
      String value = kv[1].trim();

      setState(() {
        if (key == "TREMOR") {
          tremorFreq = double.tryParse(value) ?? tremorFreq;
          tremorFreqList.add(tremorFreq);
          tremorFreqList.removeAt(0);
        } else if (key == "TREMOR_POWER") {
          tremorPower = double.tryParse(value) ?? tremorPower;
          tremorPowerList.add(tremorPower);
          tremorPowerList.removeAt(0);
        } else if (key == "MODE") {
          mode = value;
        } else if (key == "SERVOS") {
          servoStatus = value;
        } else if (key == "THR") {
          threshold = double.tryParse(value) ?? threshold;
        }
      });
    }
  }

  void _send(String cmd) {
    if (!isConnected || connection == null) return;
    connection!.output.add(Uint8List.fromList("$cmd\n".codeUnits));
  }

  void _sendServo(int id, int angle) {
    _send("SERVO $id $angle");
  }

  Widget _buildServoSlider(int index) {
    return Column(
      children: [
        Text(
          "Servo ${index + 1}: ${servoAngles[index]}°",
          style: const TextStyle(fontSize: 16),
        ),
        Slider(
          value: servoAngles[index].toDouble(),
          min: 0,
          max: 180,
          onChanged: (v) {
            setState(() => servoAngles[index] = v.toInt());
            _sendServo(index + 1, servoAngles[index]);
          },
        ),
      ],
    );
  }

  Widget _bigButton(String title, Color color, VoidCallback onPressed) {
    return Container(
      width: double.infinity,
      padding: const EdgeInsets.symmetric(horizontal: 16),
      child: ElevatedButton(
        onPressed: onPressed,
        style: ElevatedButton.styleFrom(
          backgroundColor: color,
          padding: const EdgeInsets.symmetric(vertical: 18),
          shape: RoundedRectangleBorder(
            borderRadius: BorderRadius.circular(16),
          ),
        ),
        child: Text(
          title,
          style: const TextStyle(fontSize: 20, fontWeight: FontWeight.bold),
        ),
      ),
    );
  }

  // ------------------ GRAPH WIDGET ------------------
  Widget buildGraph({
    required String title,
    required Color lineColor,
    required List<double> values,
  }) {
    return Column(
      children: [
        Text(
          title,
          style: const TextStyle(
            fontSize: 20,
            color: Colors.tealAccent,
            fontWeight: FontWeight.bold,
          ),
        ),
        const SizedBox(height: 10),
        AspectRatio(
          aspectRatio: 1.5,
          child: LineChart(
            LineChartData(
              backgroundColor: Colors.black,
              titlesData: FlTitlesData(
                show: true,
                bottomTitles: AxisTitles(
                  sideTitles: SideTitles(showTitles: false),
                ),
                leftTitles: AxisTitles(
                  sideTitles: SideTitles(showTitles: true, reservedSize: 40),
                ),
                topTitles: AxisTitles(
                  sideTitles: SideTitles(showTitles: false),
                ),
                rightTitles: AxisTitles(
                  sideTitles: SideTitles(showTitles: false),
                ),
              ),
              gridData: FlGridData(show: true),
              borderData: FlBorderData(show: true),
              lineBarsData: [
                LineChartBarData(
                  color: lineColor,
                  spots: values
                      .asMap()
                      .entries
                      .map((e) => FlSpot(e.key.toDouble(), e.value))
                      .toList(),
                  isCurved: true,
                  barWidth: 2,
                  dotData: FlDotData(show: false),
                )
              ],
            ),
          ),
        ),
      ],
    );
  }

  // ------------------ UI ------------------

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text("Hand-e-Kraft Controller"),
        backgroundColor: const Color(0xFF00695C),
        centerTitle: true,
      ),

      body: Column(
        children: [
          Expanded(
            child: SingleChildScrollView(
              padding: const EdgeInsets.all(16),
              child: Column(
                children: [
                  Icon(
                    isConnected
                        ? Icons.bluetooth_connected
                        : Icons.bluetooth_disabled,
                    color: isConnected ? Colors.tealAccent : Colors.redAccent,
                    size: 30,
                  ),
                  Text(
                    isConnecting
                        ? "Connecting..."
                        : (isConnected ? "Connected" : "Disconnected"),
                    style: const TextStyle(fontSize: 16),
                  ),
                  const SizedBox(height: 12),

                  // -------- TELEMETRY BOX --------
                  Container(
                    padding: const EdgeInsets.all(12),
                    decoration: BoxDecoration(
                      color: const Color(0xFF00796B),
                      borderRadius: BorderRadius.circular(12),
                    ),
                    child: Column(
                      children: [
                        Text("Freq: ${tremorFreq.toStringAsFixed(2)} Hz"),
                        Text("Power: ${tremorPower.toStringAsFixed(2)}"),
                        Text("Threshold: ${threshold.toStringAsFixed(2)}"),
                        Text("Mode: $mode"),
                        Text("Servos: $servoStatus"),
                      ],
                    ),
                  ),

                  const SizedBox(height: 30),

                  // -------- GRIP / RELEASE --------
                  _bigButton(
                    servoStatus == "ENGAGED" ? "RELEASE" : "GRIP",
                    servoStatus == "ENGAGED" ? Colors.red : Colors.green,
                    () {
                      if (servoStatus == "ENGAGED") {
                        _send("RELEASE");
                        setState(() => servoStatus = "RELAXED");
                      } else {
                        _send("GRIP");
                        setState(() => servoStatus = "ENGAGED");
                      }
                    },
                  ),

                  const SizedBox(height: 20),

                  // -------- AUTO MODE --------
                  _bigButton(
                    mode == "AUTO" ? "AUTO MODE ON" : "AUTO MODE OFF",
                    mode == "AUTO" ? Colors.green : Colors.orange,
                    () {
                      if (mode == "AUTO") {
                        _send("AUTO_OFF");
                        setState(() => mode = "MANUAL");
                      } else {
                        _send("AUTO_ON");
                        setState(() => mode = "AUTO");
                      }
                    },
                  ),

                  const SizedBox(height: 20),

                  // -------- STATUS --------
                  _bigButton(
                    "CHECK STATUS",
                    Colors.cyan,
                    () => _send("STATUS"),
                  ),

                  const SizedBox(height: 30),

                  // ------------- GRAPHS -------------
                  buildGraph(
                    title: "Tremor Frequency (Hz)",
                    lineColor: Colors.greenAccent,
                    values: tremorFreqList,
                  ),

                  const SizedBox(height: 30),

                  buildGraph(
                    title: "Tremor Power (Magnitude)",
                    lineColor: Colors.cyanAccent,
                    values: tremorPowerList,
                  ),

                  const SizedBox(height: 30),

                  const Text(
                    "Servo Controls",
                    style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                  ),

                  for (int i = 0; i < 5; i++) _buildServoSlider(i),
                ],
              ),
            ),
          ),

          // ----------------- FOOTER -------------------
          Container(
            width: double.infinity,
            padding: const EdgeInsets.all(10),
            decoration: BoxDecoration(
              color: Colors.black.withOpacity(0.8),
              border: const Border(
                top: BorderSide(color: Colors.tealAccent),
              ),
            ),
            child: Text(
              espStatus,
              textAlign: TextAlign.center,
              style: const TextStyle(fontSize: 14),
            ),
          ),
        ],
      ),
    );
  }
}
