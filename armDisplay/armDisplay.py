import tkinter as tk
from PIL import Image, ImageTk, ImageDraw
import random
import os
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

button_name = ["COMBAT", "EXPLORE", "ENGAGE", "RESET"]

class CRT_GUI(tk.Tk):
    def __init__(self, width=480, height=320, background_image="background.jpeg"):
        super().__init__()
        self.width = width
        self.height = height
        self.geometry(f"{width}x{height}")
        self.title("CRT Styled GUI")
        self.attributes("-fullscreen", True)
        self.configure(bg="black")
        self.bind("<Escape>", lambda event: self.attributes("-fullscreen", False))
        self.bind("<Control-q>", self.shutdown)
        self.canvas = tk.Canvas(self, width=width, height=height, bg="black", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)

        # Load and center the background image with error handling
        if background_image and os.path.exists(background_image):
            try:
                self.bg_image = Image.open(background_image).resize((width, height))
                self.bg_photo = ImageTk.PhotoImage(self.bg_image)
                self.canvas.create_image(width // 2, height // 2, image=self.bg_photo, anchor="center")
            except Exception as e:
                logger.warning(f"Failed to load background image '{background_image}': {e}. Using solid black.")
        elif background_image:
            logger.warning(f"Background image '{background_image}' not found. Using solid black.")

        # Add centered buttons with green text in the lower half of the screen
        button_y_start = int(self.height * 0.85)
        button_height = int(self.height * 0.3 / 2)
        button_width = self.width // 7

        self.buttons = []
        for i in range(4):
            button_x = ((i + 1) * (3 * self.width // 25)) + (i * button_width)
            button = tk.Button(self, text=button_name[i], command=lambda i=i: self.button_action(i),
                               bg="black", fg="lime", activebackground="green", activeforeground="black",
                               font=("Terminal", 9, "bold"))
            self.canvas.create_window(button_x, button_y_start, anchor="center", window=button, width=button_width, height=button_height)
            self.buttons.append(button)

        # Apply vignette effect (pre-rendered, non-blocking)
        self.apply_vignette()

        # Start screen flicker and noise effects
        self.screen_flicker()
        self.noise_item = None
        self.noise_overlay()

        # Start scrolling line effect
        self.scroll_line_y = 0
        self.scroll_line_effect()

    def button_action(self, button_id):
        logger.info(f"Button {button_name[button_id]} clicked")

    def shutdown(self, event=None):
        logger.info("Shutting down...")
        self.after(0, self.destroy)

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

# Example usage:
if __name__ == "__main__":
    # Set desired resolution and optional background image file path
    gui = CRT_GUI(width=480, height=320, background_image="background.jpeg")
    gui.mainloop()
