import 'dart:convert';
import 'dart:typed_data';
import 'package:flutter/material.dart';
import 'package:flutter_bluetooth_serial/flutter_bluetooth_serial.dart';
import 'package:permission_handler/permission_handler.dart';

void main() {
  runApp(MaterialApp(
    home: LEDControlApp(),
  ));
}

class LEDControlApp extends StatefulWidget {
  @override
  _LEDControlAppState createState() => _LEDControlAppState();
}

class _LEDControlAppState extends State<LEDControlApp> {
  BluetoothConnection? connection;
  bool isConnected = false;
  String receivedValue = '';

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(
          title: Text('System Control'),
        ),
        body: Center(
          child: Column(
            mainAxisAlignment: MainAxisAlignment.center,
            children: [
              ElevatedButton(
                onPressed: isConnected ? null : connectToDevice,
                child: Text('Connect to the System'),
              ),
              SizedBox(height: 20),
              ElevatedButton(
                onPressed: isConnected ? openLED : null,
                child: Text('Open System'),
              ),
              SizedBox(height: 20),
              ElevatedButton(
                onPressed: isConnected ? closeLED : null,
                child: Text('Close System'),
              ),
              SizedBox(height: 20),
              //Text('Predicted Value: $receivedValue'),
            ],
          ),
        ),
      ),
    );
  }

  void connectToDevice() async {
    // Request Bluetooth and Location permissions
    PermissionStatus bluetoothStatus = await Permission.bluetooth.request();
    PermissionStatus locationStatus = await Permission.location.request();

    if (bluetoothStatus == PermissionStatus.granted &&
        locationStatus == PermissionStatus.granted) {
      // Bluetooth permissions granted, proceed with connection

      List<BluetoothDevice> devices =
          await FlutterBluetoothSerial.instance.getBondedDevices();
      if (devices.isEmpty) {
        showDialog(
          context: context,
          builder: (BuildContext context) {
            return AlertDialog(
              title: Text('No bonded devices found'),
              content: Text('Please pair with the STM32F103C6 device first.'),
              actions: [
                TextButton(
                  onPressed: () {
                    Navigator.of(context).pop();
                  },
                  child: Text('OK'),
                ),
              ],
            );
          },
        );
        return;
      }

      BluetoothDevice device = devices[0]; // Change index based on your setup
      BluetoothConnection newConnection =
          await BluetoothConnection.toAddress(device.address);
      setState(() {
        connection = newConnection;
        isConnected = true;
      });

      connection?.input?.listen((Uint8List data) {
        int value = data[
            0]; // Assuming the received data is a single byte representing the prediction value
        setState(() {
          receivedValue = value.toString();
        });
      });
    } else {
      // Bluetooth permissions not granted, show an error dialog or handle accordingly
      showDialog(
        context: context,
        builder: (BuildContext context) {
          return AlertDialog(
            title: Text('Bluetooth Permissions Required'),
            content: Text('Please grant Bluetooth permissions to connect.'),
            actions: [
              TextButton(
                onPressed: () {
                  Navigator.of(context).pop();
                },
                child: Text('OK'),
              ),
            ],
          );
        },
      );
    }
  }

  void openLED() {
    // Send command to the microcontroller to open the LED
    Uint8List data =
        Uint8List.fromList(utf8.encode('1')); // Send 1 to open the LED
    connection?.output.add(data);
    connection?.output.allSent.then((_) {
      print('CCC: $data');

      print('System is open');
    });
  }

  void closeLED() {
    // Send command to the microcontroller to close the LED
    Uint8List data =
        Uint8List.fromList(utf8.encode('0')); // Send 0 to close the LED
    connection?.output.add(data);
    connection?.output.allSent.then((_) {
      print('System is closed');
    });
  }
}
