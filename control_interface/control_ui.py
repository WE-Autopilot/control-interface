import threading 
import rclpy

from textual.app import App, ComposeResult
from textual.containers import Container, VerticalScroll, Horizontal
from textual.widgets import Header, Footer, Input, Label

from node import AP1SystemInterfaceNode
from command_output import CommandOutput
from diagnostics_display import DiagnosticsDisplay
from visual_path import PathCanvas


def print_help(self):
    self.command_output.add_line('=' * 40)
    self.command_output.add_line('Welcome to AP1 Debug UI')
    self.command_output.add_line('Commands:')
    self.command_output.add_line('\tspeed <value>     - Set target speed (m/s)')
    self.command_output.add_line('\tlocation <x> <y>  - Set target location (m)')
    self.command_output.add_line('\tget_speed_profile - Return planned speed profile (m/s)')
    self.command_output.add_line('\treset             - Reset system + control + target speed')
    self.command_output.add_line('\tclear             - Clear screen')
    self.command_output.add_line('\thelp              - Print this screen')
    self.command_output.add_line('=' * 40)


class AP1DebugUI(App):
    CSS_PATH = 'style.css'

    def __init__(self, node: AP1SystemInterfaceNode):
        super().__init__()
        self.ros_node = node
        self.command_output = CommandOutput()

    def compose(self) -> ComposeResult:
        yield Header()

        with Horizontal(classes='half-height'):
            with Container(id='diagnostics_container', classes='pane'):
                yield Label("DIAGNOSTICS", classes='header')
                yield DiagnosticsDisplay(self.ros_node)
                yield Label("PLANNED PATH", classes='header')
                yield PathCanvas(self.ros_node)

            with Container(id='cli_container', classes='pane'):
                yield Label('COMMAND LINE INTERFACE', classes='header')
                with VerticalScroll(id='output_scroll'):
                    yield self.command_output
                yield Input(placeholder='Command...', id='command_input')

        yield Footer()

    def on_mount(self):
        threading.Thread(target=rclpy.spin, args=(self.ros_node,), daemon=True).start()
        print_help(self)
        self.query_one('#command_input', Input).focus()

    def on_input_submitted(self, event: Input.Submitted):
        cmd = event.value.strip()
        if not cmd:
            return

        self.command_output.add_line(f'> {cmd}')
        event.input.value = ''
        self.execute_command(cmd)

    def execute_command(self, cmd: str):
        try:
            parts = cmd.split()
            command = parts[0].lower()

            if command == 'help':
                print_help(self)

            elif command == 'clear':
                self.command_output.history.clear()
                self.command_output.update('')
                self.command_output.add_line('History cleared')

            elif command == 'speed':
                if len(parts) < 2:
                    self.command_output.add_line('Error! Usage: speed <value>')
                else:
                    speed = float(parts[1])
                    self.ros_node.set_target_speed(speed)
                    self.command_output.add_line(f"✓ Target speed set to {speed:.2f} m/s")

            elif command == 'location':
                if len(parts) < 3:
                    self.command_output.add_line('Error! Usage: location <x> <y>')
                else:
                    x, y = float(parts[1]), float(parts[2])
                    self.ros_node.set_target_location(x, y)
                    self.command_output.add_line(f"✓ Target location set to ({x}, {y})")

            elif command == 'get_speed_profile':
                speed_profile = self.ros_node.speed_profile
                out = ', '.join(str(s) for s in speed_profile)
                self.command_output.add_line('{ ' + out + ' }')

            elif command == "reset":
                # Try reset sim
                try:
                    self.ros_node.call_trigger_service("/pnc_sim/reset")
                    self.command_output.add_line("✓ Simulation reset triggered.")
                except Exception:
                    pass

                # Try reset control
                try:
                    self.ros_node.call_trigger_service("/control/reset")
                    self.command_output.add_line("✓ Control reset triggered.")
                except Exception:
                    pass

                # ★ NEW: reset target speed to 0
                self.ros_node.set_target_speed(0.0)
                self.command_output.add_line("✓ Target speed reset to 0.0")

            else:
                self.command_output.add_line('Command not found.')

        except Exception as e:
            self.command_output.add_line(f"Error: {str(e)}")
