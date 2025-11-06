from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import sys


class CoppeliaSim_Variables:
    def __init__(self, host="localhost", port=23000):
        self.client = RemoteAPIClient(host, port)
        self.sim = self.client.require("sim")

    def set_variable(self, name, value):
        """Set a scene signal variable"""
        try:
            self.sim.setFloatProperty(self.sim.handle_scene, f"signal.{name}", value)
            print(f"Set {name} = {value}")
            return True
        except Exception as e:
            print(f"Error: {e}")
            return False

    def get_variable(self, name):
        """Get a scene signal variable"""
        try:
            value = self.sim.getFloatProperty(
                self.sim.handle_scene, f"signal.{name}", {"noError": True}
            )
            print(f"Get {name} = {value}")
            return value
        except Exception as e:
            print(f"Error: {e}")
            return None

    def set_object_position(self, obj_name, position):
        """Set object position [x, y, z]"""
        try:
            handle = self.sim.getObject("/" + obj_name)
            self.sim.setObjectPosition(handle, -1, position)
            return True
        except Exception as e:
            print(f"Error: {e}")
            return False

    def get_object_position(self, obj_name):
        """Get object position"""
        try:
            handle = self.sim.getObject("/" + obj_name)
            pos = self.sim.getObjectPosition(handle, -1)
            return pos
        except Exception as e:
            print(f"Error: {e}")
            return None


class CoppeliaSim_CLI:
    def __init__(self):
        try:
            self.client = RemoteAPIClient()
            self.sim = self.client.require("sim")
            print("✓ Connected to CoppeliaSim\n")
        except Exception as e:
            print(f"✗ Failed to connect: {e}")
            sys.exit(1)

    def set_var(self, name, value):
        """Set variable"""
        try:
            if name == "motionMode":
                self.sim.setStringProperty(
                    self.sim.handle_scene, f"signal.{name}", (value)
                )
            else:
                self.sim.setFloatProperty(
                    self.sim.handle_scene, f"signal.{name}", float(value)
                )
            print(f"✓ {name} = {value}")
        except Exception as e:
            print(f"✗ Error: {e}")

    def get_var(self, name):
        """Get variable"""
        try:
            value = self.sim.getFloatProperty(
                self.sim.handle_scene, f"signal.{name}", {"noError": True}
            )
            if value is not None:
                print(f"✓ {name} = {value}")
            else:
                print(f"✗ Variable not found")
        except Exception as e:
            print(f"✗ Error: {e}")

    def get_pos(self, obj_name):
        """Get object position"""
        try:
            handle = self.sim.getObject("/" + obj_name)
            pos = self.sim.getObjectPosition(handle, -1)
            print(f"✓ {obj_name}: x={pos[0]:.4f}, y={pos[1]:.4f}, z={pos[2]:.4f}")
        except Exception as e:
            print(f"✗ Error: {e}")

    def set_pos(self, obj_name, x, y, z):
        """Set object position"""
        try:
            handle = self.sim.getObject("/" + obj_name)
            self.sim.setObjectPosition(handle, -1, [float(x), float(y), float(z)])
            print(f"✓ {obj_name} set to [{x}, {y}, {z}]")
        except Exception as e:
            print(f"✗ Error: {e}")

    def start(self):
        """Start simulation"""
        try:
            self.sim.startSimulation()
            print("✓ Simulation started")
        except Exception as e:
            print(f"✗ Error: {e}")

    def stop(self):
        """Stop simulation"""
        try:
            self.sim.stopSimulation()
            print("✓ Simulation stopped")
        except Exception as e:
            print(f"✗ Error: {e}")

    def pause(self):
        """Pause simulation"""
        try:
            self.sim.pauseSimulation()
            print("✓ Simulation paused")
        except Exception as e:
            print(f"✗ Error: {e}")

    def time(self):
        """Get simulation time"""
        try:
            t = self.sim.getSimulationTime()
            print(f"✓ Time: {t:.3f} s")
        except Exception as e:
            print(f"✗ Error: {e}")

    def help(self):
        """Show commands"""
        print(
            """
┌─── CoppeliaSim CLI ─────────────────────┐
│ SIMULATION:                             │
│  start                 - Start sim      │
│  stop                  - Stop sim       │
│  pause                 - Pause sim      │
│  time                  - Get time       │
│                                         │
│ VARIABLES:                              │
│  set <name> <value>    - Set variable   │
│  get <name>            - Get variable   │
│                                         │
│ OTHER:                                  │
│  help                  - Show this      │
│  quit                  - Exit           │
└─────────────────────────────────────────┘
        """
        )

    def run(self):
        """Run CLI loop"""
        self.help()
        while True:
            try:
                cmd = input(">>> ").strip().split()
                if not cmd:
                    continue

                action = cmd[0].lower()

                if action == "start":
                    self.start()
                elif action == "stop":
                    self.stop()
                elif action == "pause":
                    self.pause()
                elif action == "time":
                    self.time()
                elif action == "set" and len(cmd) >= 3:
                    self.set_var(cmd[1], cmd[2])
                elif action == "get" and len(cmd) >= 2:
                    self.get_var(cmd[1])
                elif action == "pos" and len(cmd) >= 2:
                    if len(cmd) == 2:
                        self.get_pos(cmd[1])
                    elif len(cmd) == 5:
                        self.set_pos(cmd[1], cmd[2], cmd[3], cmd[4])
                    else:
                        print("✗ Usage: pos <obj> [x y z]")
                elif action == "help":
                    self.help()
                elif action in ["quit", "exit", "q"]:
                    print("Bye!")
                    break
                else:
                    print("✗ Unknown command")

            except KeyboardInterrupt:
                print("\nCtrl+C pressed")
            except Exception as e:
                print(f"✗ {e}")


if __name__ == "__main__":
    cli = CoppeliaSim_CLI()
    cli.run()
