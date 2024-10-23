import tkinter as tk
from tkinter import ttk

def create_slider_gui():
    update_slider = {} 
    def update_label(event):
        slider = event.widget
        name = slider_names[sliders.index(slider)]
        value = (slider.get() - 300) / 100
        label_values[name].config(text=f"{name}: {value:.2f}")
        update_slider[name] = value 
    def on_exit():
        root.destroy()

    # Create the main window
    root = tk.Tk()
    root.title("Slider GUI")
    root.geometry("600x300")

    # Create a frame to hold the sliders and labels
    frame = ttk.Frame(root, padding="20")
    frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

    # Slider names
    slider_names = [
        "X Upper Bound",
        "X Lower Bound",
        "Y Upper Bound",
        "Y Lower Bound",
        "Z Upper Bound",
        "Z Lower Bound"
    ]

    # Create six sliders
    sliders = []
    for i, name in enumerate(slider_names):
        slider = ttk.Scale(frame, from_=0, to=600, orient=tk.HORIZONTAL, length=400)
        slider.set(300)
        slider.grid(row=i, column=0, columnspan=2, sticky=tk.W+tk.E, padx=10, pady=5)
        slider.bind('<Motion>', update_label)
        sliders.append(slider)
     
    # Create labels to display the slider values
    label_values = {}
    for i, name in enumerate(slider_names):
        label = ttk.Label(frame, text=f"{name}: 0.00")
        label.grid(row=i, column=2, sticky=tk.W, padx=10)
        label_values[name] = label

    # initaial value
    #for i ,name in enumerate(slider_names):
    #    sliders[i].set(initial_value[i] )
    #    label_values[name].config(text=f"{name}: {initial_value[i] - 300/ 100 :.2f}")


    # Create an Exit button
    exit_button = ttk.Button(frame, text="Exit", command=on_exit)
    exit_button.grid(row=7, column=0, columnspan=3, pady=10)

    # Run the main loop
    root.mainloop()

    # Return sliders for external access
    return update_slider

# Example of calling the function
if __name__ == "__main__":
    sliders = create_slider_gui()
    print(sliders.values() )
