import tkinter as tk
from PIL import Image, ImageTk, ImageDraw
import random
import asyncio
import logging
import threading

try:
    from bleak import BleakClient, BleakScanner
    BLE_AVAILABLE = True
except ImportError:
    BLE_AVAILABLE = False
    BleakClient = None
    BleakScanner = None

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

BUTTON_NAMES = ["COMBAT", "EXPLORE", "ENGAGE", "RESET"]
BLE_SERVICE_UUID = "09d2abe8-30ec-4519-86ff-ba0cbaf79160"
BLE_CHAR_UUID = "102d8bfe-dc7b-44d2-8cfe-0e09f2ee6107"
BLE_TARGET_NAME = "Plasma_Pistol"
BLE_SCAN_TIMEOUT = 10

class BLE_Controller:
    def __init__(self):
        self.client = None
        self.address = None
        self.connected = False
        self.loop = None

    async def discover(self, target_name=None):
        name = target_name or BLE_TARGET_NAME
        if not BLE_AVAILABLE:
            logger.warning("bleak not installed; BLE discovery skipped")
            return None
        logger.info(f"Scanning for {name}...")
        try:
            devices = await BleakScanner.discover(timeout=BLE_SCAN_TIMEOUT)
            for dev in devices:
                if name in (dev.name or ""):
                    logger.info(f"Found {name} at {dev.address}")
                    return dev.address
            logger.warning(f"No device matching '{name}' found")
            return None
        except Exception as e:
            logger.error(f"BLE scan failed: {e}")
            return None

    async def connect(self, address):
        if not BLE_AVAILABLE:
            logger.warning("bleak not installed; BLE disabled")
            return False
        try:
            self.address = address
            self.client = BleakClient(address)
            await self.client.connect()
            self.connected = True
            logger.info(f"Connected to {address}")
            return True
        except Exception as e:
            logger.error(f"BLE connection failed: {e}")
            self.connected = False
            return False

    async def disconnect(self):
        if self.client and self.connected:
            try:
                await self.client.disconnect()
            except Exception as e:
                logger.error(f"BLE disconnect error: {e}")
        self.connected = False
        self.client = None
        logger.info("BLE disconnected")

    async def send_command(self, button_index):
        if not self.connected or not self.client:
            logger.warning("BLE not connected; dropping command")
            return False
        try:
            await self.client.write_gatt_char(BLE_CHAR_UUID, bytes([button_index]))
            logger.info(f"Sent button command: {button_index}")
            return True
        except Exception as e:
            logger.error(f"BLE write failed: {e}")
            self.connected = False
            return False

class CRT_GUI(tk.Tk):
    def __init__(self, width=480, height=320, background_image="background.jpeg"):
        super().__init__()
        self.width = width
        self.height = height
        self.geometry(f"{width}x{height}")
        self.title("CRT Styled GUI")
        self.attributes("-fullscreen", True)
        self.configure(bg="black")
        self.bind("<Escape>", lambda e: self.attributes("-fullscreen", False))
        self.bind("<Control-q>", self.shutdown)
        self.canvas = tk.Canvas(self, width=width, height=height, bg="black", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)

        # BLE controller
        self.ble = BLE_Controller()
        self.ble.loop = asyncio.new_event_loop()
        self.ble_thread = threading.Thread(target=self.ble.loop.run_forever, daemon=True)
        self.ble_thread.start()

        # BLE status label
        self.ble_status = tk.Label(self, text="BLE: disconnected", fg="red", bg="black", font=("Terminal", 7))
        self.canvas.create_window(width - 60, 15, anchor="ne", window=self.ble_status)

        # Load background image with error handling
        if background_image:
            try:
                self.bg_image = Image.open(background_image).resize((width, height))
                self.bg_photo = ImageTk.PhotoImage(self.bg_image)
                self.canvas.create_image(width // 2, height // 2, image=self.bg_photo, anchor="center")
            except Exception as e:
                logger.warning(f"Failed to load background: {e}")

        # Start BLE discovery in background
        self.after(500, lambda: asyncio.run_coroutine_threadsafe(self.ble_discover(), self.ble.loop))
        self.update_ble_status()

        # Add buttons
        button_y_start = int(height * 0.85)
        button_height = int(height * 0.3 / 2)
        button_width = width // 7
        self.buttons = []
        for i in range(4):
            button_x = ((i + 1) * (3 * width // 25)) + (i * button_width)
            button = tk.Button(self, text=BUTTON_NAMES[i], command=lambda i=i: self.button_action(i),
                               bg="black", fg="lime", activebackground="green", activeforeground="black",
                               font=("Terminal", 9, "bold"))
            self.canvas.create_window(button_x, button_y_start, anchor="center", window=button, width=button_width, height=button_height)
            self.buttons.append(button)

        # Apply vignette (optimized)
        self.apply_vignette()

        # Start effects
        self.screen_flicker()
        self.noise_item = None
        self.noise_overlay()
        self.scroll_line_y = 0
        self.scroll_line_effect()

    def button_action(self, button_id):
        logger.info(f"Button {BUTTON_NAMES[button_id]} clicked")
        asyncio.run_coroutine_threadsafe(
            self.ble.send_command(button_id),
            self.ble.loop
        )

    async def ble_discover(self):
        address = await self.ble.discover()
        if address:
            connected = await self.ble.connect(address)
            if connected:
                self.after(0, lambda: self.ble_status.config(text="BLE: connected", fg="lime"))

    def update_ble_status(self):
        if self.ble.connected:
            self.ble_status.config(text="BLE: connected", fg="lime")
        else:
            self.ble_status.config(text="BLE: disconnected", fg="red")
        self.after(2000, self.update_ble_status)

    def shutdown(self, event=None):
        logger.info("Shutting down...")
        if self.ble.loop and self.ble.loop.is_running():
            asyncio.run_coroutine_threadsafe(self.ble.disconnect(), self.ble.loop)
            self.ble.loop.call_soon_threadsafe(self.ble.loop.stop)
        self.after(100, self.destroy)

    def scroll_line_effect(self):
        self.canvas.delete("scroll_line")
        self.canvas.delete("scroll_line_0")
        self.canvas.delete("scroll_line_1")
        self.canvas.create_line(0, self.scroll_line_y, self.width, self.scroll_line_y, fill="olivedrab", width=2, tags="scroll_line_0")
        self.canvas.create_line(0, self.scroll_line_y + 2, self.width, self.scroll_line_y + 2, fill="lime", width=1, tags="scroll_line")
        self.canvas.create_line(0, self.scroll_line_y + 3, self.width, self.scroll_line_y + 3, fill="olivedrab", width=2, tags="scroll_line_1")
        self.scroll_line_y += 5
        if self.scroll_line_y > self.height:
            self.scroll_line_y = 0
        self.lift_line()
        self.after(30, self.scroll_line_effect)

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
        self.after(200, self.screen_flicker)

    def noise_overlay(self):
        noise = Image.new("RGBA", (self.width, self.height), (0, 0, 0, 0))
        draw = ImageDraw.Draw(noise)
        for _ in range(300):
            x = random.randint(0, self.width - 1)
            y = random.randint(0, self.height - 1)
            color = random.choice([(200, 200, 200, 60), (100, 100, 100, 60)])
            draw.point((x, y), fill=color)
        self.noise_image = ImageTk.PhotoImage(noise)
        if self.noise_item is not None:
            self.canvas.delete(self.noise_item)
        self.noise_item = self.canvas.create_image(0, 0, image=self.noise_image, anchor="nw")
        self.after(100, self.noise_overlay)

if __name__ == "__main__":
    gui = CRT_GUI(width=480, height=320, background_image="background.jpeg")
    gui.mainloop()
