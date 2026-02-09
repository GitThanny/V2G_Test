
import time
import threading
import sys

# Try to import python-can, but provide helpful error if missing
try:
    import can
except ImportError:
    print("Error: 'python-can' library is required but not found.")
    print("Please install it using: pip install python-can")
    # We will let the class definition fail or stub it if we want to allow import without usage
    # For now, let's allow import but fail on __init__
    can = None

class CanCharger():
    """
    Implementation of ChargerInterface for Phoenix Contact Charger module via CAN bus.
    Uses 'python-can' library for CAN communication.
    """

    # Protocol Constants
    DEVICE_NO_P2P = 0x0A
    DEVICE_NO_GROUP = 0x0B
    TARGET_ADDR_BROADCAST = 0x3F
    
    # Priority/Error Code
    PRIO_NORMAL = 0x00
    
    # Source Address (Controller)
    SOURCE_ADDR_DEFAULT = 0xF0

    def __init__(self, interface='virtual', channel='vcan0', bitrate=125000):
        if can is None:
            raise ImportError("python-can library not installed.")

        self.channel = channel
        self.bitrate = bitrate
        self.interface_type = interface
        self.bus = None
        self.is_connected = False
        
        # Internal state
        self.evse_max_voltage = 0
        self.evse_min_voltage = 0
        self.evse_max_current = 0
        self.evse_min_current = 0
        self.evse_max_power = 0
        self.evse_present_voltage = 0.0
        self.evse_present_current = 0.0
        
        self.ev_max_voltage = 0
        self.ev_min_voltage = 0
        self.ev_max_current = 0
        self.ev_min_current = 0
        self.ev_max_power = 0
        
        self.started = False
        self._stop_event = threading.Event()
        self._receive_thread = None

        # Try to open connection
        try:
            # Note: For many interfaces, bitrate is set at OS level, but we pass it anyway
            self.bus = can.Bus(interface=self.interface_type, channel=self.channel, bitrate=self.bitrate)
            self.is_connected = True
            print(f"Connected to CAN interface: {self.interface_type}/{self.channel}")
        except Exception as e:
            print(f"Failed to open CAN connection: {e}")

        if self.is_connected:
            self._start_receive_thread()

    def _start_receive_thread(self):
        self._receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._receive_thread.start()

    def _receive_loop(self):
        last_request_time = 0
        last_keepalive_time = 0
        REQUEST_INTERVAL = 0.2  # Send CAN read requests every 200ms
        KEEPALIVE_INTERVAL = 5.0  # Send keep-alive every 5 seconds

        while not self._stop_event.is_set():
            try:
                now = time.time()
                
                # Periodically send CAN read requests for voltage/current
                if self.started and (now - last_request_time) >= REQUEST_INTERVAL:
                    self._request_present_values()
                    last_request_time = now

                # Periodically send keep-alive (operational readiness)
                if self.started and (now - last_keepalive_time) >= KEEPALIVE_INTERVAL:
                    data = [0x11, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0]
                    self._send_command(0x24, data)
                    print(f"[CAN] Keep-alive sent")
                    last_keepalive_time = now

                # Receive CAN responses (short timeout to allow periodic sends)
                msg = self.bus.recv(timeout=0.05)
                if msg:
                    self._process_frame(msg)
            except Exception as e:
                print(f"Error in receive loop: {e}")
                time.sleep(0.1)

    def _request_present_values(self):
        """Send CAN read requests for present voltage and current."""
        # Read System Voltage: 0x23 0x10 0x01
        data_v = [0x10, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        self._send_command(0x23, data_v, target_addr=self.TARGET_ADDR_BROADCAST)
        # Read System Current: 0x23 0x10 0x02
        data_c = [0x10, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        self._send_command(0x23, data_c, target_addr=self.TARGET_ADDR_BROADCAST)
                
    def _process_frame(self, msg):
        """
        Parses received CAN frame and updates internal state.
        Expects a can.Message object.
        """
        if not msg.is_extended_id:
            return

        ident = msg.arbitration_id
        # Parse Identifier
        # Ident: Prio(3) Dev(4) Cmd(6) Target(8) Source(8)
        # 28-26    25-22  21-16   15-8      7-0
        
        # error_code = (ident >> 26) & 0x07
        # device_no = (ident >> 22) & 0x0F
        # command_no = (ident >> 16) & 0x3F
        # target_addr = (ident >> 8) & 0xFF
        # source_addr = ident & 0xFF
        
        payload = msg.data
        if len(payload) < 8:
            return

        byte0 = payload[0]
        byte1 = payload[1]

        # Parsing Logic based on protocol examples
        # Response to System Voltage Read: 0x10 0x01 ...
        if byte0 == 0x10 and byte1 == 0x01:
            # Bytes 4-7 is value in mV
            val_bytes = bytes(payload[4:8])
            voltage_mv = int.from_bytes(val_bytes, byteorder='big', signed=False)
            self.evse_present_voltage = voltage_mv / 1000.0
            # print(f"Updated Voltage: {self.evse_present_voltage} V")

        # Response to System Current Read: 0x10 0x02 ...
        elif byte0 == 0x10 and byte1 == 0x02:
            val_bytes = bytes(payload[4:8])
            current_ma = int.from_bytes(val_bytes, byteorder='big', signed=False)
            self.evse_present_current = current_ma / 1000.0
            # print(f"Updated Current: {self.evse_present_current} A")

    def _build_identifier(self, error_code, device_no, command_no, target_addr, source_addr):
        ident = 0
        ident |= (error_code & 0x07) << 26
        ident |= (device_no & 0x0F) << 22
        ident |= (command_no & 0x3F) << 16
        ident |= (target_addr & 0xFF) << 8
        ident |= (source_addr & 0xFF)
        return ident

    def _send_command(self, command_no, data, target_addr=0x00, device_no=DEVICE_NO_P2P):
        if not self.is_connected:
            return

        ident = self._build_identifier(
            self.PRIO_NORMAL, 
            device_no, 
            command_no, 
            target_addr, 
            self.SOURCE_ADDR_DEFAULT
        )
        
        try:
            msg = can.Message(
                arbitration_id=ident, 
                data=data, 
                is_extended_id=True
            )
            self.bus.send(msg)
        except Exception as e:
            print(f"Error sending CAN frame: {e}")

    def start(self):
        """Starts the charger operation (Enable operational readiness)."""
        # Command 0x24: Enable operational readiness
        # Data: 0x11 0x10 00 00 00 00 00 A0 (A0 = ON)
        data = [0x11, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0]
        self._send_command(0x24, data)
        self.started = True

    def stop(self):
        """Stops the charger operation (Disable operational readiness)."""
        # Command 0x24: Disable operational readiness
        # Data: 0x11 0x10 00 00 00 00 00 A1 (A1 = OFF)
        data = [0x11, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA1]
        self._send_command(0x24, data)
        self.started = False
        
        # Also set voltage/current to 0 safety
        self.setEvTargetVoltage(0)
        self.setEvTargetCurrent(0)

    def close(self):
        """Stops receive thread and closes bus."""
        self._stop_event.set()
        if self._receive_thread:
            self._receive_thread.join(timeout=2.0)
        if self.bus:
            self.bus.shutdown()
        self.is_connected = False

    # --- Setters for EVSE Capabilities ---
    def setEvseMaxCurrent(self, value):
        self.evse_max_current = value

    def setEvseMinCurrent(self, value):
        self.evse_min_current = value

    def setEvseMaxVoltage(self, value):
        self.evse_max_voltage = value

    def setEvseMinVoltage(self, value):
        self.evse_min_voltage = value

    def setEvseMaxPower(self, value):
        self.evse_max_power = value

    def setEvseDeltaVoltage(self, value):
        pass

    def setEvseDeltaCurrent(self, value):
        pass

    # --- Setters for EV Limits ---
    def setEvMaxCurrent(self, value):
        self.ev_max_current = value

    def setEvMinCurrent(self, value):
        self.ev_min_current = value

    def setEvMaxVoltage(self, value):
        self.ev_max_voltage = value

    def setEvMinVoltage(self, value):
        self.ev_min_voltage = value

    def setEvMinPower(self, value):
        pass

    def setEvMaxPower(self, value):
        self.ev_max_power = value

    # --- Dynamic Targets ---
    def setEvTargetVoltage(self, voltage):
        """
        Sets the system voltage on DC side.
        Protocol: 0x24 0x10 0x01 [4 bytes val]
        """
        if self.isVoltageLimitExceeded(voltage):
            return False
        
        voltage_mv = int(voltage * 1000) 
        val_bytes = voltage_mv.to_bytes(4, byteorder='big', signed=False)
        data = [0x10, 0x01, 0x00, 0x00, val_bytes[0], val_bytes[1], val_bytes[2], val_bytes[3]]
        
        self._send_command(0x24, data, target_addr=self.TARGET_ADDR_BROADCAST, device_no=self.DEVICE_NO_P2P)
        return True

    def setEvTargetCurrent(self, current):
        """
        Sets the system current on DC side.
        Protocol: 0x24 0x10 0x02 [4 bytes val]
        """
        if self.isCurrentLimitExceeded(current):
            return False
            
        current_ma = int(current * 1000)
        val_bytes = current_ma.to_bytes(4, byteorder='big', signed=False)
        data = [0x10, 0x02, 0x00, 0x00, val_bytes[0], val_bytes[1], val_bytes[2], val_bytes[3]]
        
        self._send_command(0x24, data, target_addr=self.TARGET_ADDR_BROADCAST)
        return True

    # --- Getters ---
    def getEvseMaxCurrent(self): return self.evse_max_current
    def getEvseMinCurrent(self): return self.evse_min_current
    def getEvseMaxVoltage(self): return self.evse_max_voltage
    def getEvseMinVoltage(self): return self.evse_min_voltage
    def getEvseMaxPower(self): return self.evse_max_power
    def getEvseDeltaVoltage(self): return 0 
    def getEvseDeltaCurrent(self): return 0
    def getEvMaxCurrent(self): return self.ev_max_current
    def getEvMinCurrent(self): return self.ev_min_current
    def getEvMaxVoltage(self): return self.ev_max_voltage
    def getEvMinVoltage(self): return self.ev_min_voltage
    def getEvMinPower(self): return 0
    def getEvMaxPower(self): return self.ev_max_power

    # --- Real-time Values ---
    def getEvsePresentVoltage(self):
        # Request update: 0x23 0x10 0x01 (System Voltage)
        data = [0x10, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        self._send_command(0x23, data, target_addr=self.TARGET_ADDR_BROADCAST)
        return self.evse_present_voltage

    def getEvsePresentCurrent(self):
        # Request update: 0x23 0x10 0x02 (System Current)
        data = [0x10, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        self._send_command(0x23, data, target_addr=self.TARGET_ADDR_BROADCAST)
        return self.evse_present_current

    # --- Safety Checks ---
    def isVoltageLimitExceeded(self, voltage):
        return voltage > self.evse_max_voltage or (self.evse_min_voltage > 0 and voltage < self.evse_min_voltage)

    def isCurrentLimitExceeded(self, current):
        return current > self.evse_max_current or (self.evse_min_current > 0 and current < self.evse_min_current)

    def isPowerLimitExceeded(self, power):
        return power > self.evse_max_power

if __name__ == "__main__":
    print("Initializing CanCharger with virtual interface...")
    # NOTE: This requires a virtual CAN interface named 'vcan0' to be up.
    # sudo modprobe vcan
    # sudo ip link add dev vcan0 type vcan
    # sudo ip link set up vcan0
    try:
        charger = CanCharger(interface='virtual', channel='vcan0')
        charger.start()
        time.sleep(1)
        charger.stop()
        charger.close()
        print("Test Complete.")
    except ImportError:
        print("Skipping test: python-can not installed.")
    except Exception as e:
        print(f"Test failed (expected if vcan0 not up): {e}")
