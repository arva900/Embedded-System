import os
import tkinter as tk
from tkinter import filedialog
import threading

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
from Comm import *
# ---------------- Globals ----------------
stop_state = threading.Event()
done = threading.Event()
current_state = 0  # 0=idle, 1..5 states

latest_angle = start_angle = 0
latest_distance = start_distance = prev_distance = filtered_distance = 0
spike_dists = []
light_distance=0

root = tk.Tk()
angle_entry = None
set_btn = None
angle_var = None
state_var = None
canvas = None
ax = None
button_texts = ["Objects Detector", "Telemeter", "Light Source Detector", "Objects+Light", "File Mode", "Light Calib",
                "Idle"]

# ------------------- Thread handling -------------------
def start_state(state):
    global current_state
    if state==7:
        state_var.set("state 0 = Idle")
        state=0
    else: state_var.set(f"state{state} = {button_texts[state-1]}")
    if done.is_set():
        current_state = state
        ax.clear()
        ax.set_position([-0.15, -0.15, 1.3, 1.3])
        ax.set_theta_direction(1)
        ax.set_thetalim(-0.05, np.pi + 0.05)
    else:root.after(0, lambda f=state:start_state(f))  # Comm still busy → check again after 5 ms
    if current_state == 2:
        set_btn.grid()
        angle_entry.grid()
    else:
        set_btn.grid_remove()
        angle_entry.grid_remove()

def select_state():
    global latest_angle, latest_distance,filtered_distance,light_distance,current_state
    prev_state = 0
    payload=[0,0,0]
    while not stop_state.is_set():
        done.clear()
        if current_state!=prev_state : #changing states
            print("\nstart state ", current_state)
            if 5>prev_state>0 : #when first state - msp just waiting to rcv- no read needed
                payload = read_frame(3,1)  # [echo_hi, echo_lo, angle]
            elif prev_state==5: send_byte(bytes([ACK]))
            prev_state = current_state
            latest_distance=filtered_distance=0
            time.sleep(2)#msp exit state
            if current_state:
                if not send_frame([current_state]): prev_state=0 # send new state
                else: print(f"!!!!!!!!! state changed to {prev_state}\n")
            if current_state in (3,4) : ADCcalib()
            continue
        elif current_state==prev_state==0:
            done.set()
            time.sleep(0.01)
            continue
        elif current_state in (1,2,3,4):
            payload = read_frame(3, 0)# [echo_hi, echo_lo, angle]
            if payload==[0,0,0] : current_state=prev_state=0
            while current_state==2 and not send_frame([latest_angle]):
                print("Failed to send angle")
                #time.sleep(0.1)
                continue
        elif current_state==5:
            filename = filedialog.askopenfilename(
                initialdir=r"C:\Users\shaha\PycharmProjects\DCSFinalProject",  # Start in project folder
                title="Select file to send",
                filetypes=[("Text files", "*.txt")]  # only .txt
                           #("All files", "*.*"))  # fallback: show all
            )
            if not filename :
                current_state = 0
                continue
            filename = os.path.basename(filename)
            # First byte is file type (0 txt,1 script)
            # Second byte is file number
            header = "0" if filename[0].lower() == "t" else "1"
            header = header + ''.join(filter(str.isdigit, filename))
            try:
                with open(filename, 'r') as file:
                    if header[0] == "0": send_txt(file,header)
                    else: send_script(file,header)
                    ack = read_byte()
                    if ack==ACK : print(f"File {filename} sent successfully")
                    else:print(f"unsuccessful {filename} !!")
            except Exception as e:print(f"Error: {e}")
            if header[0] =='1':
                current_state=prev_state=7
                #set_timout(None)
            continue
        elif current_state == 6:current_state=prev_state=0
        elif current_state==7:
            opc = read_frame(3, 0)
            print(f"opc={opc}")
            if opc[0] == 6:
                latest_angle=opc[1]
                end_angle=opc[1]+1
            elif opc[0] == 7:
                latest_angle = opc[1]
                end_angle=opc[2]
            elif opc[0] == 8:current_state=prev_state=5
            else :time.sleep(1)
        if current_state in (1,2,3,4,7):
            done.clear()
            raw_distance = raw = 0
            if current_state==1:raw_distance, latest_angle, raw = getOBJdist(payload)
            elif current_state==2:raw_distance, _, raw = getOBJdist(payload)
            elif current_state==3:
                latest_distance, latest_angle, raw = getLIGHTdist(payload)
                continue
            elif current_state==4:
                    light_distance, _, raw = getLIGHTdist(payload)#adc
                    payload = read_frame(3, 0) ##ultrasonic
                    raw_distance, latest_angle, raw = getOBJdist(payload)
                    if light_distance<=40:
                        latest_distance=0
                        root.after(0, update_radar)
                        continue
            elif current_state==7 and (opc[0] == 7 or opc[0] == 6):
                spike_dists = []
                latest_distance=0
                while latest_angle < end_angle:
                    payload = read_frame(3, 0)
                    raw_distance, latest_angle, raw = getOBJdist(payload)
                    latest_distance = smthDIST(raw_distance)
                    root.after(0, update_radar)  # refresh every rcv
                    print(f"[STATE{current_state},{opc}] Distance:{latest_distance}cm | "
                          f"Angle:{latest_angle:3d}°\n")
                latest_angle = 181
                continue
            else:continue
            done.set()
            latest_distance = smthDIST(raw_distance)
            print(f"[STATE {current_state}] Distance:{latest_distance}cm | "
                  f"Angle:{latest_angle:3d}° | "
                  f"raw value:{raw:5d} \n")
            if current_state==2:root.after(5, update_radar)  # refresh every rcv
            else:root.after(0, update_radar)

def smthDIST(distance):
    global filtered_distance,spike_dists
    if filtered_distance == 0: filtered_distance = distance
    elif distance == 0: return int(filtered_distance)
    else:
        # Spike rejection: ignore sudden jumps larger than threshold
        if abs(distance - filtered_distance) > 5:
            if len(spike_dists)>0 and abs(distance - np.mean(spike_dists))>5: spike_dists=[]
            spike_dists.append(int(distance))
            if len(spike_dists) >= 4:  # new object?
                filtered_distance = np.mean(spike_dists)
            print(f"Spike{len(spike_dists)} ignored: {distance} cm")
        else:
            spike_dists = []
            # Exponential Moving Average smoothing
            filtered_distance = 0.7 * distance + (1 - 0.7) * filtered_distance
    return int(filtered_distance)
# ------------------- GUI functions -------------------

def print_length(ang1_rad,dist1,ang2_rad,dist2):
    # Convert polar → Cartesian
    x1, y1 = dist1 * np.cos(ang1_rad), dist1 * np.sin(ang1_rad)
    x2, y2 = dist2 * np.cos(ang2_rad), dist2 * np.sin(ang2_rad)
    # Length of object
    l = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    # Annotate length at midpoint
    mid_ang = (ang1_rad + ang2_rad) / 2
    max_dist = max([dist1,dist2])
    ax.text(mid_ang, max_dist+1 , f"({max_dist:3d}cm,{int(np.rad2deg(ang1_rad))}->{latest_angle}°,{int(l)}cm) )",
           ha="center", va="center", fontsize=10, color="black")


# ------------------- Radar plot ------------------#
def update_radar():
    # Update radar
    global prev_distance,latest_distance,start_distance,latest_angle,start_angle,spike_dists,light_distance
    if (latest_angle==181 and current_state in (1,3,4)) or current_state in (2,7):
        if current_state in (1,3,4): start_angle=prev_distance=start_distance=0
        for line in ax.lines[:]:line.remove()
        for txt in ax.texts[:]:txt.remove()
    angle_var.set(f"{latest_angle}")
    rad_angle = np.radians(latest_angle)
    if len(spike_dists)==4 and latest_distance>0 and current_state ==1:#start a new object (and finish the prev)
        print_length(start_angle,start_distance,rad_angle,prev_distance)#end obj
        start_distance=latest_distance #start new obj
        start_angle = rad_angle
        spike_dists=[]
    elif current_state in (2,3,7):
        ax.text(rad_angle, latest_distance+1 ,f"({int(latest_distance)}cm,{latest_angle}°)")
        if start_angle!=rad_angle:
            start_angle=rad_angle
            spike_dists=[]

    elif current_state==4 and light_distance>40: #no light
        if len(spike_dists) == 4 and latest_distance > 0:
            print_length(start_angle,start_distance,rad_angle,prev_distance)#end obj
            start_distance = latest_distance  # start new obj
            start_angle = rad_angle
            spike_dists = []
    elif current_state==4 and light_distance<=40:# and start_distance != latest_distance: # light
        #print_length(start_angle, start_distance, rad_angle, prev_distance)  # end obj
        ax.text(rad_angle, light_distance+1,f"({int(light_distance)}cm,{latest_angle}°)" ,color="green")

    prev_distance=latest_distance
    # Plot the line
    if latest_distance>0:
        color = 'red' if current_state in (1,2,4,7) else 'green'
        ax.plot([rad_angle], [latest_distance], marker='o', linewidth=1, color=color)
    elif current_state==4 and latest_distance==0 :#light detected in stat4
            ax.plot([rad_angle], [light_distance], marker='o', linewidth=1, color='green')
    canvas.draw()

def set_angle():
    global latest_angle,angle_entry
    try:
        val = int(angle_entry.get())
        if 0 <= val <= 180:
            if done.is_set():latest_angle = val
            else:root.after(0, set_angle)  # Comm still busy → check again after 5 ms
        else: print("Error", "Angle must be 0-180")
    except ValueError:print("Error", "Invalid angle")
# ------------------- GUI init -------------------
def init_GUI():
    global root,state_var,set_btn,angle_entry,angle_var,ax,canvas
    root.title("Final Project GUI")
    root.geometry("1100x700")
    # Command buttons
    button_frame = tk.Frame(root)
    button_frame.pack()
    for i in range(len(button_texts)):
        hex_value = format((i + 1) * 2, '01x')  # convert to hex and ensure 2 digits
        color = f"#0f90{hex_value}f"
        width = 20 if i<len(button_texts)-2 else 10
        btn = tk.Button(button_frame,
                        text=button_texts[i],
                        font = ("Arial", 10, "bold"),
                        command=lambda f=i+1: start_state(f),
                        bg=color,
                        fg="black",
                        width=width)
        btn.grid(row=0, column=i, pady=5, padx=2,ipady=10)

    # Angle selection
    angle_frame = tk.Frame(root, bg="#f0f4f7")
    angle_frame.pack()

    state_var = tk.StringVar(value="state 0 = Idle")
    state_disp = tk.Label(angle_frame, textvariable=state_var, bg="#f0f4f7", fg="#0f90ff", font=("Arial", 14, "bold"))
    state_disp.grid(row=0, column=0, padx=5, ipady=10)

    angle_label = tk.Label(angle_frame, text="Angle (0-180):", bg="#f0f4f7", fg="black", font=("Arial", 18))
    angle_label.grid(row=0, column=3, padx=5,ipady=10)

    angle_var = tk.StringVar(value="0°")
    angle_disp = tk.Label(angle_frame, textvariable=angle_var, bg="#f0f4f7", fg="#1F3eF0", font=("Arial", 20, "bold"))
    angle_disp.grid(row=0, column=4, padx=5, ipady=10)

    angle_entry = tk.Entry(angle_frame, width=10, justify=tk.CENTER, bg="#f0f4f7", font=("Arial", 16))
    angle_entry.grid(row=0, column=5, padx=5,ipady=5)

    angle_entry.insert(0, "0")
    angle_entry.grid_remove()

    set_btn = tk.Button(angle_frame, text="Set",
                          command=lambda: set_angle(),
                          bg="#16a085", fg="black", width=10)
    set_btn.grid(row=0, column=6, padx=5,ipady=10)
    set_btn.grid_remove()


    fig = plt.Figure(figsize=(6, 6))
    ax = fig.add_subplot(111, polar=True)
    ax.set_position([-0.15, -0.15, 1.3, 1.3])
    ax.set_theta_direction(1)
    ax.set_thetalim(-0.05, np.pi+0.05)
    #ax.set_rlim(0, 450)  # Set range from 0 to 450 cm
    canvas = FigureCanvasTkAgg(fig, master=root)
    canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
    canvas.draw()
    update_radar()
    root.mainloop()
