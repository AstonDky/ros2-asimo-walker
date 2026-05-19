import tkinter as tk
from tkinter import ttk

from .teleop_profiles import requested_profile_name, resolve_profile_from_input


class TeleopGuiApp:
    def __init__(self, command_buffer, status_provider=None, shutdown_callback=None):
        self.command_buffer = command_buffer
        self.status_provider = status_provider
        self.shutdown_callback = shutdown_callback
        self._pressed = set()
        self.key_widgets = {}

        self.root = tk.Tk()
        self.root.title("Asti ASIMO-style Teleop")
        self.root.protocol("WM_DELETE_WINDOW", self._close)
        self.root.geometry("680x520")

        self.key_vars = {
            "W": tk.StringVar(value="released"),
            "S": tk.StringVar(value="released"),
            "A": tk.StringVar(value="released"),
            "D": tk.StringVar(value="released"),
            "Q": tk.StringVar(value="released"),
            "E": tk.StringVar(value="released"),
            "Shift": tk.StringVar(value="released"),
        }
        self.requested_var = tk.StringVar(value="idle")
        self.active_var = tk.StringVar(value="idle")
        self.status_var = tk.StringVar(value="Idle / safe hold.")
        self.pause_var = tk.StringVar(value="false")
        self.estop_var = tk.StringVar(value="false")
        self.state_var = tk.StringVar(value="WAIT")
        self.pitch_var = tk.StringVar(value="-")
        self.roll_var = tk.StringVar(value="-")

        self._build()
        self.root.bind_all("<KeyPress>", self._on_key_press)
        self.root.bind_all("<KeyRelease>", self._on_key_release)
        self.root.after(200, self.root.focus_force)
        self.root.after(100, self._refresh)

    def run(self) -> None:
        self.root.mainloop()

    def _build(self) -> None:
        main = ttk.Frame(self.root, padding=12)
        main.grid(row=0, column=0, sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        ttk.Label(main, text="Keyboard Teleop Profiles", font=("", 14, "bold")).grid(row=0, column=0, columnspan=3, sticky="w")
        ttk.Label(main, text="Press once to latch. Press W again to stop after the current safe step.").grid(
            row=1, column=0, columnspan=3, sticky="w", pady=(2, 10)
        )

        rows = (
            ("W", "Forward", "enabled"),
            ("S", "Backward", "reserved"),
            ("A", "Turn left", "reserved"),
            ("D", "Turn right", "reserved"),
            ("Q", "Waist left", "reserved"),
            ("E", "Waist right", "reserved"),
            ("Shift", "Fast modifier", "reserved"),
        )
        for index, (key, action, availability) in enumerate(rows, start=2):
            key_label = tk.Label(
                main,
                text=key,
                width=8,
                height=2,
                bd=1,
                relief="raised",
                bg="#e8eaed",
                fg="#202124",
                font=("", 11, "bold"),
            )
            key_label.grid(row=index, column=0, sticky="ew", pady=3, padx=(0, 8))
            ttk.Label(main, text=f"{action}, {availability}", width=28).grid(row=index, column=1, sticky="w", pady=2)
            state_label = tk.Label(
                main,
                textvariable=self.key_vars[key],
                width=14,
                height=2,
                bd=1,
                relief="solid",
                bg="#f8f9fa",
                fg="#3c4043",
            )
            state_label.grid(row=index, column=2, sticky="ew", pady=3)
            self.key_widgets[key] = (key_label, state_label)

        controls = ttk.Frame(main)
        controls.grid(row=9, column=0, columnspan=3, sticky="ew", pady=(10, 4))
        ttk.Label(controls, text="Space: pause/resume").grid(row=0, column=0, sticky="w", padx=(0, 16))
        ttk.Label(controls, text="R: reset keys").grid(row=0, column=1, sticky="w", padx=(0, 16))
        ttk.Label(controls, text="Esc: emergency stop").grid(row=0, column=2, sticky="w")

        status = ttk.LabelFrame(main, text="Status", padding=8)
        status.grid(row=10, column=0, columnspan=3, sticky="ew", pady=(8, 0))
        status.columnconfigure(1, weight=1)

        status_rows = (
            ("requested_profile", self.requested_var),
            ("active_profile", self.active_var),
            ("status_message", self.status_var),
            ("pause", self.pause_var),
            ("emergency_stop", self.estop_var),
            ("state", self.state_var),
            ("pitch", self.pitch_var),
            ("roll", self.roll_var),
        )
        for index, (label, var) in enumerate(status_rows):
            ttk.Label(status, text=label, width=18).grid(row=index, column=0, sticky="w", pady=2)
            ttk.Label(status, textvariable=var, wraplength=460).grid(row=index, column=1, sticky="w", pady=2)

    def _on_key_press(self, event) -> None:
        keysym = event.keysym
        if keysym in self._pressed:
            return
        self._pressed.add(keysym)

        lower = keysym.lower()
        if lower in ("w", "s", "a", "d", "q", "e"):
            self.command_buffer.press_key(lower)
        elif keysym in ("Shift_L", "Shift_R"):
            self.command_buffer.press_key("shift")
        elif keysym == "space":
            self.command_buffer.toggle_pause()
        elif lower == "r":
            self.command_buffer.reset_keys()
        elif keysym == "Escape":
            self.command_buffer.set_emergency_stop()
        self._refresh(schedule_next=False)

    def _on_key_release(self, event) -> None:
        keysym = event.keysym
        self._pressed.discard(keysym)
        self._refresh(schedule_next=False)

    def _refresh(self, schedule_next=True) -> None:
        state = self.command_buffer.snapshot()
        key_values = {
            "W": state.key_w,
            "S": state.key_s,
            "A": state.key_a,
            "D": state.key_d,
            "Q": state.key_q,
            "E": state.key_e,
            "Shift": state.shift,
        }
        for key, pressed in key_values.items():
            self.key_vars[key].set("ACTIVE" if pressed else "idle")
            self._style_key(key, pressed)

        self.pause_var.set(str(state.pause).lower())
        self.estop_var.set(str(state.emergency_stop).lower())

        _, fallback_message = resolve_profile_from_input(state)
        self.requested_var.set(requested_profile_name(state))
        self.status_var.set(fallback_message)

        if self.status_provider is not None:
            node_status = self.status_provider()
            self.requested_var.set(node_status.get("requested_profile", self.requested_var.get()))
            self.active_var.set(node_status.get("active_profile", self.active_var.get()))
            self.status_var.set(node_status.get("status_message", self.status_var.get()))
            self.state_var.set(node_status.get("state", self.state_var.get()))
            self.pitch_var.set(node_status.get("pitch", self.pitch_var.get()))
            self.roll_var.set(node_status.get("roll", self.roll_var.get()))

        if schedule_next:
            self.root.after(100, self._refresh)

    def _style_key(self, key: str, active: bool) -> None:
        widgets = self.key_widgets.get(key)
        if widgets is None:
            return
        key_label, state_label = widgets
        if not active:
            key_label.configure(bg="#e8eaed", fg="#202124", relief="raised")
            state_label.configure(bg="#f8f9fa", fg="#3c4043")
            return
        if key == "W":
            key_label.configure(bg="#146c43", fg="white", relief="sunken")
            state_label.configure(bg="#0f5132", fg="white")
        else:
            key_label.configure(bg="#8a5a00", fg="white", relief="sunken")
            state_label.configure(bg="#664d03", fg="white")

    def _close(self) -> None:
        if self.shutdown_callback is not None:
            self.shutdown_callback()
        self.root.destroy()
