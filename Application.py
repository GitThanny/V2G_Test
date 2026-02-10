import argparse
import json
import sys
import os

'''
sudo -E .venv/bin/python3 Application.py eth -i eth0 -m c4:93:00:48:AC:F0 -r EVSE --auto
sudo -E .venv/bin/python3 Application.py spi -i spidev0.0 -m c4:93:00:48:AC:F0 -r EVSE --auto
sudo -E nice -n -19 .venv/bin/python3 Application.py eth -i eth0 -m c4:93:00:48:AC:F0 -r EVSE --auto
sudo ip link set can2 up type can bitrate 125000 loopback on
sudo ip link set can2 up type can bitrate 125000
sudo ip link set can2 down
source .venv/bin/activate
'''

from Evse import *
from Ev import *

if __name__ == "__main__":
    WHITEBBET_DEFAULT_MAC = "00:01:01:63:77:33"
    parser = argparse.ArgumentParser(description='Codico Whitebeet reference implementation.')
    parser.add_argument('interface_type', type=str, choices=('eth', 'spi'), help='Type of the interface through which the Whitebeet is connected. ("eth" or "spi").')
    parser.add_argument('-i', '--interface', type=str, required=True, help='This is the name of the interface where the Whitebeet is connected to (i.e. for eth "eth0" or spi "0").')
    parser.add_argument('-m', '--mac', type=str, help='This is the MAC address of the ethernet interface of the Whitebeet (i.e. "{}").'.format(WHITEBBET_DEFAULT_MAC))
    parser.add_argument('-r', '--role', type=str, choices=('EVSE', 'EV'), required=True, help='This is the role of the Whitebeet. "EV" for EV mode and "EVSE" for EVSE mode')
    parser.add_argument('-c', '--config', type=str, help='Path to EV configuration file. Defaults to ./ev.json.\nA MAC present in the config file will override a MAC provided with -m argument.', nargs='?', const="./ev.json")
    parser.add_argument('-ec', '--evse-config', type=str, help='Path to EVSE configuration file. Defaults to ./evse.json.\nA MAC present in the config file will override a MAC provided with -m argument.', nargs='?', const="./evse.json")
    parser.add_argument('-p', '--portmirror', help='Enables port mirror.', action='store_true')
    parser.add_argument('--auto', action='store_true', help='Automatically authorize the EV connection for EVSE mode.')
    parser.add_argument('--ocpp-url', type=str, help='WebSocket URL for OCPP CSMS (e.g., ws://localhost:9000/CP_1).')
    parser.add_argument('--ocpp-id', type=str, help='Charge Point ID for OCPP.')
    parser.add_argument('--ocpp-version', type=str, choices=('1.6', '2.0.1', '2.1'), default='1.6', help='OCPP Protocol Version (default: 1.6).')
    args = parser.parse_args()

        # If no MAC address was given set it to the default MAC address of the Whitebeet
    if args.interface_type == "eth" and args.mac is None:
        args.mac = WHITEBBET_DEFAULT_MAC

    print('Welcome to Codico Whitebeet {} reference implementation'.format(args.role))

    # role is EV
    if(args.role == "EV"):
        mac = args.mac
        config = None
        # Load configuration from json
        if args.config is not None:
            try:
                with open(args.config, 'r') as configFile:
                    config = json.load(configFile)
                    if 'mac' in config and config['mac']:
                        mac = config['mac'] # Config file MAC overrides command-line
            except FileNotFoundError:
                print(f"Configuration file {args.config} not found. Using default configuration.")
            except json.JSONDecodeError:
                print(f"Error decoding {args.config}. The file is likely malformed. Using default configuration.")
                config = None # Ensure config is None if JSON is bad
                
        # If no MAC was provided by command line or config file, use the default.
        if args.interface_type in ["eth", "eth_raw"] and mac is None:
            mac = WHITEBBET_DEFAULT_MAC

        if mac is None and args.interface_type in ["eth", "eth_raw"]:
            print("Error: A MAC address must be provided for an ethernet interface via command line (-m) or a config file (-c).")
            exit(1)

        with Ev(args.interface_type, args.interface, mac) as ev:
            # Apply config to ev
            if config is not None:
                print("EV configuration: " + str(config))
                ev.load(config)

            # Start the EVSE loop
            ev.whitebeet.networkConfigSetPortMirrorState(args.portmirror)
            ev.loop()
            print("EV loop finished")

    elif(args.role == 'EVSE'):
        evse_mac = args.mac
        evse_config_data = None
        if args.evse_config is not None:
            try:
                with open(args.evse_config, 'r') as configFile:
                    evse_config_data = json.load(configFile)
                    if 'mac' in evse_config_data:
                        evse_mac = evse_config_data['mac']
            except FileNotFoundError:
                print(f"Configuration file {args.evse_config} not found. Using default EVSE configuration.")
            except json.JSONDecodeError:
                print(f"Error decoding {args.evse_config}. The file is likely malformed. Using default EVSE configuration.")
                evse_config_data = None # Ensure config is None if JSON is bad


        with Evse(args.interface_type, args.interface, evse_mac, auto_authorize=args.auto) as evse:
            if evse_config_data and 'CanCharger' in evse_config_data:
                CanCharger_config = evse_config_data['CanCharger']
                evse.getCanCharger().setEvseDeltaVoltage(CanCharger_config.get('delta_voltage', 0.5))
                evse.getCanCharger().setEvseDeltaCurrent(CanCharger_config.get('delta_current', 0.05))
                evse.getCanCharger().setEvseMaxVoltage(CanCharger_config.get('max_voltage', 100))
                evse.getCanCharger().setEvseMaxCurrent(CanCharger_config.get('max_current', 1))
                evse.getCanCharger().setEvseMaxPower(CanCharger_config.get('max_power', 25000))
            else:
                # Default CanCharger parameters
                evse.getCanCharger().setEvseDeltaVoltage(0.5)
                evse.getCanCharger().setEvseDeltaCurrent(0.05)
                evse.getCanCharger().setEvseMaxVoltage(100)
                evse.getCanCharger().setEvseMaxCurrent(1)
                evse.getCanCharger().setEvseMaxPower(25000)

            # Start the CanCharger
            evse.getCanCharger().start()

            if evse_config_data and 'schedule' in evse_config_data:
                schedule = evse_config_data['schedule']
                evse.setSchedule(schedule)
            else:
                # Default schedule
                schedule = {
                    "code": 0,
                    "schedule_tuples": [{
                        'schedule_tuple_id': 1,
                        'schedules':[
                            {
                                "start": 0,
                                "interval": 0,
                                "power": 25000
                            },
                            {
                                "start": 1800,
                                "interval": 0,
                                "power": 18750
                            },
                            {
                                "start": 3600,
                                "interval": 82800,
                                "power": 12500
                            }
                        ]
                    }]
                }
                evse.setSchedule(schedule)

            # Start the EVSE loop
            evse.whitebeet.networkConfigSetPortMirrorState(args.portmirror)
            evse.loop()
            print("EVSE loop finished")

    print("Goodbye!")

