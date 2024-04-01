import board
import neopixel

# Define the number of pixels in the strip
num_pixels = 60  # Change this according to your LED strip

# Define the GPIO pin connected to the data line
pixel_pin = board.D38  # Change this according to your setup

# Create NeoPixel object with appropriate configuration
pixels = neopixel.NeoPixel(pixel_pin, num_pixels, auto_write=False)

# Define some colors
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
WHITE = (255, 255, 255)
OFF = (0, 0, 0)

# Function to set color of the entire strip
def fill(color):
    pixels.fill(color)
    pixels.show()

# Example usage
if __name__ == "__main__":
    try:
        fill(RED)  # Set the entire strip to red
        # You can change the color and other effects here
    except KeyboardInterrupt:
        fill(OFF)  # Turn off LEDs before exiting
