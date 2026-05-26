import tkinter as tk
from PIL import Image, ImageTk, ImageDraw
import random
import asyncio
import logging
import threading
import json
import os
import time
from copy import deepcopy

try:
    from bleak import BleakClient, BleakScanner
    BLE_AVAILABLE = True
except ImportError:
    BLE_AVAILABLE = False
    BleakClient = None
    BleakScanner = None

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

DEFAULT_CONFIG = {
    "button_labels": ["CONQUEROR", "PROTECTOR", "AEGIS", "CLEANSE"],
    "button_colors": {
        "bg": "black",
        "fg": "lime",
        "active_bg": "green",
        "active_fg": "black",
        "font": "Terminal",
        "font_size": 9
    },
    "effects": {
        "scanline_interval": 30,
        "flicker_interval": 200,
        "noise_interval": 100,
        "noise_dots": 300
    },
    "ble": {
        "targets": ["Plasma_Pistol", "LEDGoggles", "Servo_Skull", "TechPriest_Flamer"],
        "uuids": {
            "Plasma_Pistol": {
                "service": "09d2abe8-30ec-4519-86ff-ba0cbaf79160",
                "char": "102d8bfe-dc7b-44d2-8cfe-0e09f2ee6107"
            },
            "TechPriest_Flamer": {
                "service": "09d2abe9-30ec-4519-86ff-ba0cbaf79160",
                "char": "102d8bff-dc7b-44d2-8cfe-0e09f2ee6107"
            },
            "LEDGoggles": {
                "service": "09d2abea-30ec-4519-86ff-ba0cbaf79160",
                "char": "102d8bf0-dc7b-44d2-8cfe-0e09f2ee6107"
            },
            "Servo_Skull": {
                "service": "09d2abeb-30ec-4519-86ff-ba0cbaf79160",
                "char": "102d8bf1-dc7b-44d2-8cfe-0e09f2ee6107"
            }
        },
        "service_uuid": "09d2abe8-30ec-4519-86ff-ba0cbaf79160",
        "char_uuid": "102d8bfe-dc7b-44d2-8cfe-0e09f2ee6107",
        "scan_timeout": 10,
        "reconnect_delay": 5,
        "max_reconnect_attempts": 3
    },
    "display": {
        "width": 480,
        "height": 320,
        "background_image": "background.jpeg"
    }
}

def deep_merge(base, override):
    """Recursively merge override into base dict."""
    result = base.copy()
    for k, v in override.items():
        if k in result and isinstance(result[k], dict) and isinstance(v, dict):
            result[k] = deep_merge(result[k], v)
        else:
            result[k] = v
    return result

def load_config(path="config.json"):
    if os.path.exists(path):
        try:
            with open(path) as f:
                user = json.load(f)
            cfg = deep_merge(DEFAULT_CONFIG, user)
            logger.info(f"Loaded config from {path}")
            return cfg
        except Exception as e:
            logger.warning(f"Failed to load config: {e}, using defaults")
    return DEFAULT_CONFIG.copy()

class BLE_Device:
    def __init__(self, name, address, service_uuid, char_uuid):
        self.name = name
        self.address = address
        self.service_uuid = service_uuid
        self.char_uuid = char_uuid
        self.client = None
        self.connected = False

    async def connect(self):
        if not BLE_AVAILABLE:
            return False
        try:
            self.client = BleakClient(self.address)
            await self.client.connect()
            self.connected = True
            logger.info(f"Connected to {self.name} at {self.address}")
            return True
        except Exception as e:
            logger.error(f"Failed to connect to {self.name}: {e}")
            self.connected = False
            return False

    async def disconnect(self):
        if self.client and self.connected:
            try:
                await self.client.disconnect()
            except Exception as e:
                logger.error(f"Disconnect error for {self.name}: {e}")
        self.connected = False
        self.client = None

    async def send_command(self, button_index):
        if not self.connected or not self.client:
            return False
        try:
            await self.client.write_gatt_char(self.char_uuid, bytes([button_index]))
            return True
        except Exception as e:
            logger.error(f"Write failed for {self.name}: {e}")
            self.connected = False
            return False

class BLE_Controller:
    def __init__(self, config):
        self.config = config.get("ble", {})
        self.targets = self.config.get("targets", ["Plasma_Pistol"])
        self.uuids = self.config.get("uuids", DEFAULT_CONFIG["ble"]["uuids"])
        self.service_uuid = self.config.get("service_uuid", DEFAULT_CONFIG["ble"]["service_uuid"])
        self.char_uuid = self.config.get("char_uuid", DEFAULT_CONFIG["ble"]["char_uuid"])
        self.scan_timeout = self.config.get("scan_timeout", 10)
        self.reconnect_delay = self.config.get("reconnect_delay", 5)
        self.max_reconnect_attempts = self.config.get("max_reconnect_attempts", 3)
        self.devices = {}
        self.loop = None
        self._reconnect_task = None
        self._reconnect_attempts = 0
        self._running = False
        self._devices_lock = threading.Lock()

    def _get_uuids_for_target(self, target):
        """Get service/char UUIDs for a specific target, falling back to defaults."""
        if target in self.uuids:
            return self.uuids[target]["service"], self.uuids[target]["char"]
        return self.service_uuid, self.char_uuid

    async def discover_all(self):
        if not BLE_AVAILABLE:
            logger.warning("bleak not installed; BLE disabled")
            return {}
        logger.info(f"Scanning for targets: {self.targets}")
        try:
            async with BleakScanner() as scanner:
                devices = await scanner.discover(timeout=self.scan_timeout)
            found = {}
            for dev in devices:
                dev_name = dev.name or ""
                for target in self.targets:
                    if dev_name == target:
                        svc_uuid, chr_uuid = self._get_uuids_for_target(target)
                        d = BLE_Device(target, dev.address, svc_uuid, chr_uuid)
                        found[target] = d
                        logger.info(f"Found {target} at {dev.address}")
                        break
            return found
        except Exception as e:
            logger.error(f"BLE scan failed: {e}")
            return {}

    async def connect_all(self):
        connected = 0
        for name, dev in self.devices.items():
            if await dev.connect():
                connected += 1
        self._reconnect_attempts = 0
        return connected

    async def disconnect_all(self):
        self._running = False
        for name, dev in self.devices.items():
            await dev.disconnect()
        logger.info("All BLE devices disconnected")

    async def send_to_all(self, button_index):
        coros = [dev.send_command(button_index) for dev in self.devices.values()]
        results = await asyncio.gather(*coros, return_exceptions=True)
        named = []
        for (name, _), r in zip(self.devices.items(), results):
            ok = r if isinstance(r, bool) else False
            named.append((name, ok))
            logger.info(f"Sent {button_index} to {name}: {'OK' if ok else 'FAIL'}")
        return named

    async def reconnect_loop(self):
        self._running = True
        while self._running:
            disconnected = [n for n, d in self.devices.items() if not d.connected]
            if not disconnected:
                self._reconnect_attempts = 0
                await asyncio.sleep(self.reconnect_delay)
                continue
            self._reconnect_attempts += 1
            if self._reconnect_attempts > self.max_reconnect_attempts:
                logger.warning(f"Max reconnect attempts ({self.max_reconnect_attempts}) reached, backing off...")
                await asyncio.sleep(30)
                self._reconnect_attempts = 0
                continue
            logger.info(f"Reconnect attempt {self._reconnect_attempts}/{self.max_reconnect_attempts}")
            found = await self.discover_all()
            for name, dev in found.items():
                if name in self.devices:
                    self.devices[name].address = dev.address
            await self.connect_all()
            await asyncio.sleep(self.reconnect_delay)

    def start_reconnect(self):
        if not self._running and self.devices:
            self._running = True
            self._reconnect_task = asyncio.run_coroutine_threadsafe(
                self.reconnect_loop(), self.loop
            )

class CRT_GUI(tk.Tk):
    def __init__(self, app_config=None):
        self._app_config = app_config or load_config()
        disp = self._app_config.get("display", {})
        width = disp.get("width", 480)
        height = disp.get("height", 320)
        bg_image = disp.get("background_image", "background.jpeg")
        self.width = width
        self.height = height
        super().__init__()
        self.geometry(f"{width}x{height}")
        self.title("Tech Priest Arm Display")
        self.attributes("-fullscreen", True)
        self.configure(bg="black")
        self.bind("<Escape>", lambda e: self.attributes("-fullscreen", False))
        self.bind("<Control-q>", self.shutdown)
        self.canvas = tk.Canvas(self, width=width, height=height, bg="black", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)

        self.ble = BLE_Controller(self._app_config)
        self.ble.loop = asyncio.new_event_loop()
        self.ble_thread = threading.Thread(target=self.ble.loop.run_forever, daemon=True)
        self.ble_thread.start()

        self.ble_status = tk.Label(self, text="BLE: scanning", fg="yellow", bg="black", font=("Terminal", 7))
        self.canvas.create_window(width - 60, 15, anchor="ne", window=self.ble_status)

        self.connected_count = tk.Label(self, text="", fg="lime", bg="black", font=("Terminal", 7))
        self.canvas.create_window(width - 60, 30, anchor="ne", window=self.connected_count)

        if bg_image:
            try:
                self.bg_image = Image.open(bg_image).resize((width, height))
                self.bg_photo = ImageTk.PhotoImage(self.bg_image)
                self.canvas.create_image(width // 2, height // 2, image=self.bg_photo, anchor="center")
            except Exception as e:
                logger.warning(f"Failed to load background: {e}")

        self.after(500, lambda: asyncio.run_coroutine_threadsafe(self.ble_discover(), self.ble.loop))
        self.update_ble_status()

        self.create_buttons()
        self.apply_vignette()
        self.start_effects()

    def create_buttons(self):
        labels = self._app_config.get("button_labels", DEFAULT_CONFIG["button_labels"])
        colors = self._app_config.get("button_colors", DEFAULT_CONFIG["button_colors"])
        button_y_start = int(self.height * 0.85)
        button_height = int(self.height * 0.3 / 2)
        num = len(labels)
        gap = self.width // (num * 2 + 1)
        button_width = gap
        self.buttons = []
        for i in range(num):
            button_x = gap * (2 * i + 1)
            button = tk.Button(
                self, text=labels[i], command=lambda i=i: self.button_action(i),
                bg=colors.get("bg", "black"), fg=colors.get("fg", "lime"),
                activebackground=colors.get("active_bg", "green"),
                activeforeground=colors.get("active_fg", "black"),
                font=(colors.get("font", "Terminal"), colors.get("font_size", 9), "bold")
            )
            self.canvas.create_window(button_x, button_y_start, anchor="center", window=button, width=button_width, height=button_height)
            self.buttons.append(button)

    def start_effects(self):
        effects = self._app_config.get("effects", DEFAULT_CONFIG["effects"])
        self.scanline_interval = effects.get("scanline_interval", 30)
        self.flicker_interval = effects.get("flicker_interval", 200)
        self.noise_interval = effects.get("noise_interval", 100)
        self.noise_dots = effects.get("noise_dots", 300)
        self.noise_item = None
        self.scroll_line_y = 0
        self.main_update()

    def main_update(self):
        now = time.monotonic()
        if not hasattr(self, '_last_scanline'):
            self._last_scanline = 0
        if not hasattr(self, '_last_flicker'):
            self._last_flicker = 0
        if not hasattr(self, '_last_noise'):
            self._last_noise = 0

        if now - self._last_scanline >= self.scanline_interval / 1000:
            self.scroll_line_effect()
            self._last_scanline = now

        if now - self._last_flicker >= self.flicker_interval / 1000:
            self.screen_flicker()
            self._last_flicker = now

        if now - self._last_noise >= self.noise_interval / 1000:
            self.noise_overlay()
            self._last_noise = now

        self.after(16, self.main_update)

    def button_action(self, button_id):
        labels = self._app_config.get("button_labels", DEFAULT_CONFIG["button_labels"])
        logger.info(f"Button {labels[button_id]} clicked")
        asyncio.run_coroutine_threadsafe(
            self.ble.send_to_all(button_id),
            self.ble.loop
        )

    async def ble_discover(self):
        found = await self.ble.discover_all()
        if found:
            with self.ble._devices_lock:
                self.ble.devices = found
            count = await self.ble.connect_all()
            self.after(0, lambda: self.update_status_label(count))
            if count > 0:
                self.ble.start_reconnect()
        else:
            self.after(0, lambda: self.ble_status.config(text="BLE: not found", fg="red"))

    def update_status_label(self, count):
        with self.ble._devices_lock:
            total = len(self.ble.devices)
            connected_names = ', '.join(n for n, d in self.ble.devices.items() if d.connected)
        self.ble_status.config(text=f"BLE: {count}/{total}", fg="lime" if count == total else "yellow")
        self.connected_count.config(text=f"Connected: {connected_names}")

    def update_ble_status(self):
        with self.ble._devices_lock:
            connected = sum(1 for d in self.ble.devices.values() if d.connected)
        self.update_status_label(connected)
        self.after(2000, self.update_ble_status)

    def shutdown(self, event=None):
        logger.info("Shutting down...")
        self._shutdown = True
        if self.ble.loop and self.ble.loop.is_running():
            fut = asyncio.run_coroutine_threadsafe(self.ble.disconnect_all(), self.ble.loop)
            fut.add_done_callback(lambda _: self.ble.loop.call_soon_threadsafe(self.ble.loop.stop))
        self.after(100, self.destroy)

    def scroll_line_effect(self):
        if not hasattr(self, '_scroll_line_ids'):
            self._scroll_line_ids = (
                self.canvas.create_line(0, 0, 0, 0, fill="olivedrab", width=2, tags="scroll_line_0"),
                self.canvas.create_line(0, 0, 0, 0, fill="lime", width=1, tags="scroll_line"),
                self.canvas.create_line(0, 0, 0, 0, fill="olivedrab", width=2, tags="scroll_line_1")
            )
        y = self.scroll_line_y
        self.canvas.coords(self._scroll_line_ids[0], 0, y, self.width, y)
        self.canvas.coords(self._scroll_line_ids[1], 0, y + 2, self.width, y + 2)
        self.canvas.coords(self._scroll_line_ids[2], 0, y + 3, self.width, y + 3)
        self.scroll_line_y += 5
        if self.scroll_line_y > self.height:
            self.scroll_line_y = 0
        self.lift_line()

    def lift_line(self):
        self.canvas.tag_raise("scroll_line")
        self.canvas.tag_raise("scroll_line_0")
        self.canvas.tag_raise("scroll_line_1")

    def apply_vignette(self):
        vignette = Image.new("RGBA", (self.width, self.height), (0, 0, 0, 0))
        draw = ImageDraw.Draw(vignette)
        max_radius = (self.width + self.height) // 2
        step = max(1, max_radius // 60)
        for i in range(0, max_radius, step):
            color_intensity = int(255 * (i / max_radius) ** 2)
            draw.ellipse(
                [(self.width // 2 - i, self.height // 2 - i), (self.width // 2 + i, self.height // 2 + i)],
                outline=(0, 100, 0, color_intensity)
            )
        self.vignette_image = ImageTk.PhotoImage(vignette)
        self.canvas.create_image(0, 0, image=self.vignette_image, anchor="nw")

    def screen_flicker(self):
        flicker_color = "black" if random.choice([True, False]) else "#0a0a0a"
        self.canvas.configure(bg=flicker_color)

    def noise_overlay(self):
        if not hasattr(self, '_noise_img'):
            self._noise_img = Image.new("RGBA", (self.width, self.height), (0, 0, 0, 0))
            self.noise_item = self.canvas.create_image(0, 0, image=self._noise_img, anchor="nw")
        draw = ImageDraw.Draw(self._noise_img)
        self._noise_img.putdata([(0, 0, 0, 0)] * (self.width * self.height))
        for _ in range(self.noise_dots):
            x = random.randint(0, self.width - 1)
            y = random.randint(0, self.height - 1)
            color = random.choice([(200, 200, 200, 60), (100, 100, 100, 60)])
            draw.point((x, y), fill=color)
        self.noise_image = ImageTk.PhotoImage(self._noise_img)
        self.canvas.itemconfig(self.noise_item, image=self.noise_image)

if __name__ == "__main__":
    config = load_config()
    gui = CRT_GUI(config)
    gui.mainloop()
