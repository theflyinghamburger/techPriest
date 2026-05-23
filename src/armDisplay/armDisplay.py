import tkinter as tk
from PIL import Image, ImageTk, ImageDraw
import random

button_name = ["COMBAT", "EXPLORE", "ENGAGE", "RESET"]

class CRT_GUI(tk.Tk):
    def __init__(self, width=480, height=320, background_image="background.jpeg"):
        super().__init__()
        self.geometry(f"{width}x{height}")
        self.title("CRT Styled GUI")
        self.attributes("-fullscreen", True)
        self.configure(bg="black")  # Set window background to black for dark mode
        self.bind("<Escape>", lambda event: self.attributes("-fullscreen", False))
        # Set up the canvas with a black background
        self.canvas = tk.Canvas(self, width=width, height=height, bg="black", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)
        
        # Load and center the background image
        if background_image:
            self.bg_image = Image.open(background_image).resize((width, height))
            self.bg_photo = ImageTk.PhotoImage(self.bg_image)
            self.canvas.create_image(width // 2, height // 2, image=self.bg_photo, anchor="center")

        # Add centered buttons with green text in the lower half of the screen
        button_y_start = int(height * 0.85)
        button_height = int(height * 0.3 / 2)
        button_width = width // 7  # Adjust button width for spacing
        
        self.buttons = []
        for i in range(4):
            # Calculate x position to center buttons with even spacing
            button_x = ((i + 1) * (3*width // 25)) + (i * button_width)
            button = tk.Button(self, text=button_name[i], command=lambda i=i: self.button_action(i),
                               bg="black", fg="lime", activebackground="green", activeforeground="black", 
                               font=("Terminal", 9, "bold"))
            self.canvas.create_window(button_x, button_y_start, anchor="center", window=button, width=button_width, height=button_height)
            self.buttons.append(button)
        
        # Start scrolling line effect
        self.scroll_line_y = 0  # Initial y position of the line
        self.scroll_line_effect()

        # Apply vignette effect
        self.apply_vignette(width, height)

        # Start screen flicker and noise effects
        self.screen_flicker()
        self.noise_overlay(width, height)

    def button_action(self, button_id):
        print(f"Button {button_id + 1} clicked")
        # Insert the specific Bluetooth actions for each button here

    def scroll_line_effect(self):
        # Clear previous line
        self.canvas.delete("scroll_line")
        self.canvas.delete("scroll_line_0")
        self.canvas.delete("scroll_line_1")
        # Draw the line at the current position on top of everything
        self.canvas.create_line(0, self.scroll_line_y, self.winfo_width(), self.scroll_line_y, fill="olivedrab", width=2, tags="scroll_line_0")
        self.canvas.create_line(0, (self.scroll_line_y+2), self.winfo_width(), self.scroll_line_y+2, fill="lime", width=1, tags="scroll_line")
        self.canvas.create_line(0, (self.scroll_line_y+3), self.winfo_width(), self.scroll_line_y+3, fill="olivedrab", width=2, tags="scroll_line_1")
        # Update line position
        self.scroll_line_y += 5  # Adjust speed by changing this value
        if self.scroll_line_y > self.winfo_height():  # Reset line to top if it reaches the bottom
            self.scroll_line_y = 0
        
        # Redraw the line after buttons, making sure it overlays them
        self.lift_line()
        
        # Repeat this function after a short delay to create the scrolling effect
        self.after(30, self.scroll_line_effect)

    def lift_line(self):
        # Lift the line above all other widgets
        self.canvas.tag_raise("scroll_line")
        self.canvas.tag_raise("scroll_line_0")
        self.canvas.tag_raise("scroll_line_1")

    def apply_vignette(self, width, height):
        # Create a vignette effect by overlaying a gradient in dark green on the canvas
        vignette = Image.new("RGBA", (width, height), (0, 0, 0, 0))
        draw = ImageDraw.Draw(vignette)
        
        # Gradient effect (dark green towards the edges)
        max_radius = (width + height) // 2
        for i in range(max_radius):
            color_intensity = int(255 * (i / max_radius) ** 2)
            draw.ellipse(
                [(width // 2 - i, height // 2 - i), (width // 2 + i, height // 2 + i)],
                outline=(0, 100, 0, color_intensity)
            )
        
        # Convert the vignette to a Tkinter-compatible image and overlay it
        vignette = vignette.resize((width, height), Image.LANCZOS)
        self.vignette_image = ImageTk.PhotoImage(vignette)
        self.canvas.create_image(0, 0, image=self.vignette_image, anchor="nw")

    def screen_flicker(self):
        # Apply a flicker effect by briefly adjusting the background brightness
        # flicker_color = "black" if random.choice([True, False]) else "#0a0a0a"
        flicker_color = "black" if random.choice([True, False]) else "green"
        self.canvas.configure(bg=flicker_color)
        self.after(200, self.screen_flicker)

    def noise_overlay(self, width, height):
        # Create a random noise overlay
        noise = Image.new("RGBA", (width, height), (0, 0, 0, 0))
        draw = ImageDraw.Draw(noise)
        
        # Draw random white and gray dots to simulate noise
        for _ in range(300):  # Adjust number of dots for effect
            x = random.randint(0, width - 1)
            y = random.randint(0, height - 1)
            color = random.choice([(200, 200, 200, 60), (100, 100, 100, 60)])
            draw.point((x, y), fill=color)
        
        # Convert the noise overlay to a Tkinter-compatible image and place it on the canvas
        self.noise_image = ImageTk.PhotoImage(noise)
        self.canvas.create_image(0, 0, image=self.noise_image, anchor="nw", tags="noise")
        
        # Refresh the noise overlay periodically for dynamic effect
        self.after(100, self.noise_overlay, width, height)

# Example usage:
if __name__ == "__main__":
    # Set desired resolution and optional background image file path
    gui = CRT_GUI(width=480, height=320, background_image="background.jpeg")
    gui.mainloop()
