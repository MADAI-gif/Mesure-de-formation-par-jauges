# === Début du code optimisé ===
import tkinter as tk
from tkinter import ttk, messagebox
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
import csv, os, time, sys
from datetime import datetime
from PIL import Image, ImageTk
import os
import serial
import serial.tools.list_ports

# --------------------------- VARS ----------------------------
ser = None
running = False
start_time = time.time()
data_log, hist_data = [], []
angle_deg = 0.0
masse = force = deformation = tension = 0.0
last_def = 0.0

# ---------------------- SERIAL COM ---------------------------
def actualiser_ports():
    ports = [p.device for p in serial.tools.list_ports.comports()]
    port_menu['values'] = ports
    port_menu.set(ports[0] if ports else "")
    
def toggle_running():
    global running
    running = not running
    if running:
        start_button.config(text="Stop", bg="red")
        update_values()
    else:
        start_button.config(text="Start", bg="green")
        
        
def resource_path(relative_path):
    base_path = os.path.abspath(os.path.dirname(__file__))
    return os.path.join(base_path, relative_path)
        

def connecter_serial():
    global ser
    port = port_menu.get()
    if not port:
        messagebox.showwarning("Port non sélectionné", "Sélectionne un port.")
        return
    try:
        ser = serial.Serial(port, 115200, timeout=0.01)
        connexion_button.config(text="Déconnecter", command=deconnecter_serial)
        mode_menu.config(state="readonly")
        angle_scale.config(state="normal")
        status_label.config(image=green_dot)
        status_label.image = green_dot
        if auto_start_var.get(): toggle_running()
    except serial.SerialException:
        messagebox.showerror("Serial", f"Impossible d'ouvrir {port}")

def deconnecter_serial():
    global ser
    if ser and ser.is_open: ser.close()
    ser = None
    connexion_button.config(text="Connecter", command=connecter_serial)
    mode_menu.config(state="disabled")
    angle_scale.config(state="disabled")
    status_label.config(image=red_dot)
    status_label.image = red_dot

def send_mode(event=None):
    global angle_deg
    if not ser: return
    m = 1 if mode_menu.get() == "Manuel" else 2
    a = int(angle_scale.get()) if m == 1 else 0
    ser.write(f"{m},{a}e".encode())
    angle_deg = a if m == 1 else angle_deg
    angle_label.config(text=f"{angle_deg:.1f} °")
    update_radar()

# ---------------------- LECTURE UART ------------------------
def update_values():
    global running, angle_deg, masse, force, deformation, tension
    if not running: return

    if ser and ser.in_waiting:
        try:
            raw = ser.read(ser.in_waiting).decode(errors='ignore')
            for line in raw.splitlines():
                if line.startswith("V:"): tension = float(line[2:])
                elif line.startswith("M:"): masse = float(line[2:])
                elif line.startswith("D:"): deformation = float(line[2:])
                elif line.startswith("F:"): force = float(line[2:])
                elif line.startswith("A:"): angle_deg = float(line[2:])
        except ValueError: pass

    t = time.time() - start_time
    chrono_label.config(text=f"Temps : {int(t)} s")
    data_log.append([round(t,2), angle_deg, masse, force, deformation, tension])
    hist_data.append(tension)

    angle_label.config(text=f"{angle_deg:.1f}°")
    label_masse.config(text=f"Masse : {masse:.4f} g")
    label_force.config(text=f"Force : {force:.4f} N")
    label_deformation.config(text=f"Déformation : {deformation:.6f}")
    label_tension.config(text=f"Tension : {tension:.4f} V")

    update_graph()
    update_radar()
    dessiner_lame(deformation)

    root.after(10, update_values)

# ------------------------ UI GRAPHIQUES -----------------------
def update_radar():
    radar_ax.clear()
    radar_ax.set_facecolor('black')
    radar_ax.plot([0, np.deg2rad(angle_deg)], [0, 2], color='red', linewidth=2)
    radar_ax.set_ylim(0, 2)
    radar_ax.set_yticklabels([])
    radar_ax.set_xticks(np.linspace(0, np.pi, 13))
    radar_ax.set_xticklabels([f"{i*15}°" for i in range(13)], color='#ff7518')
    radar_ax.set_thetamin(0); radar_ax.set_thetamax(180)
    radar_canvas.draw()
    

# === Ajout des fonctions de zoom ===
def on_press(event):
    global press_x
    if event.inaxes == graph_ax:
        press_x = event.xdata

def on_release(event):
    global press_x
    if event.inaxes == graph_ax and hasattr(on_release, 'press_x'):
        x1, x2 = sorted([on_release.press_x, event.xdata])
        graph_ax.set_xlim(x1, x2)
        graph_canvas.draw()
    press_x = None

def on_scroll(event):
    if event.inaxes == graph_ax:
        # Zoom X
        cur_xlim = graph_ax.get_xlim()
        xdata = event.xdata
        scale_factor = 1.2 if event.button == 'up' else 1/1.2

        new_width = (cur_xlim[1] - cur_xlim[0]) * scale_factor
        rel_x = (xdata - cur_xlim[0]) / (cur_xlim[1] - cur_xlim[0])
        new_xmin = xdata - new_width * rel_x
        new_xmax = xdata + new_width * (1 - rel_x)

        # Zoom Y
        cur_ylim = graph_ax.get_ylim()
        ydata = event.ydata
        new_height = (cur_ylim[1] - cur_ylim[0]) * scale_factor
        rel_y = (ydata - cur_ylim[0]) / (cur_ylim[1] - cur_ylim[0])
        new_ymin = ydata - new_height * rel_y
        new_ymax = ydata + new_height * (1 - rel_y)

        graph_ax.set_xlim(new_xmin, new_xmax)
        graph_ax.set_ylim(new_ymin, new_ymax)
        graph_canvas.draw()


def reset_zoom(event=None):
    graph_ax.set_xlim(0, len(hist_data))
    graph_ax.set_ylim(0, 4)
    graph_canvas.draw()


def update_graph():
    graph_ax.clear()
    if hist_data:
        graph_ax.plot(range(len(hist_data)), hist_data, label='Tension')
    graph_ax.set_xlabel("Échantillons")
    graph_ax.set_ylabel("Tension (V)")
    graph_ax.set_ylim(0, 4)
    graph_ax.set_yticks(np.arange(0, 4.1, 0.1))
    graph_ax.tick_params(axis='y', labelsize=8)
    graph_ax.grid(True)
    graph_ax.legend()
    graph_canvas.draw()
    

def reset_graph():
    hist_data.clear(); data_log.clear()
    graph_ax.clear(); graph_canvas.draw()
    
    
    
def dessiner_lame(defor):
    global last_def
    
    # Calcul de la déformation avec lissage
    alpha = 0.2
    amplitude = (1 - alpha) * last_def + alpha * defor * 1000
    last_def = amplitude
    amplitude = max(min(amplitude, 20), -20)
    
    # Effacer le canvas existant
    for widget in lame_canvas.winfo_children():
        widget.destroy()
    
    # Paramètres de la lame (largeur réduite)
    longueur = 10
    largeur = 0.2  # Réduit à 0.5 au lieu de 2 pour une lame plus fine
    nb_points = 50
    
    x = np.linspace(0, longueur, nb_points)
    y = np.linspace(-largeur/2, largeur/2, 3)  # Moins de points en largeur
    
    # Création de la figure 3D
    fig = plt.Figure(figsize=(6, 4), dpi=100)
    ax = fig.add_subplot(111, projection='3d')
    
    # Poutre verticale bleue (dimensions ajustées)
    ax.plot([0, 0], [0, 0], [0, 5], color='blue', linewidth=6)  # Épaisseur réduite
    
    # Surface de la lame plus étroite
    X, Y = np.meshgrid(x, y)
    Z = -amplitude/20 * (X / longueur)**2 * (3 - 2 * X / longueur) + 5
    ax.plot_surface(X, Y, Z, color='green', alpha=0.9, rstride=1, cstride=1, linewidth=0)
    
    # Flèche rouge plus fine
    arrow_dir = -1 if amplitude >= 0 else 1
    ax.quiver(x[-1], 0, Z[1,-1], 0, 0, arrow_dir, 
              length=1, color='red', arrow_length_ratio=0.2, linewidth=1)
    
    # Configuration des axes
    ax.set_xlim(0, longueur)
    ax.set_ylim(-1, 1)  # Garde un peu d'espace même si la lame est fine
    ax.set_zlim(0, 6)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_zticks([])
    ax.set_facecolor('white')
    ax.grid(False)
    
    # Vue initiale
    ax.view_init(elev=25, azim=-45)
    
    # Intégration dans Tkinter
    canvas_3d = FigureCanvasTkAgg(fig, master=lame_canvas)
    canvas_3d.draw()
    canvas_3d.get_tk_widget().pack(fill='both', expand=True)
    
    # Conservation des références
    dessiner_lame.canvas_3d = canvas_3d
    dessiner_lame.fig = fig


def reset_3d_view():
    if hasattr(dessiner_lame, 'fig'):
        ax = dessiner_lame.fig.axes[0]
        ax.view_init(elev=25, azim=-45)
        dessiner_lame.canvas_3d.draw()

    
    
# ------------------------- SAUVEGARDE ------------------------
def save_data():
    base_dir = os.path.dirname(os.path.abspath(__file__))  # Dossier du script actuel
    fld = os.path.join(base_dir, "IHM_Donnees_sauvegarde")  # Sous-dossier local
    os.makedirs(fld, exist_ok=True)
    fn = os.path.join(fld, f"log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
    with open(fn, 'w', newline='') as f:
        w = csv.writer(f, delimiter=';')
        w.writerow(['t(s)','Angle(°)','Masse(g)','Force(N)','Déf','Tension(V)'])
        w.writerows(data_log)
    messagebox.showinfo("Sauvegarde", "Fichier créé :\n"+fn)   

def show_hist():
    w=tk.Toplevel(root); w.title("Historique")
    txt=tk.Text(w); txt.pack(fill='both',expand=True)
    for i, d in enumerate(data_log[-20:],1):
        txt.insert('end',f"{i}. t={d[0]}s Angle={d[1]:.1f}° M={d[2]:.1f}g F={d[3]:.3f}N D={d[4]:.6f} T={d[5]:.2f}V\n")

# ------------------------ FENETRE ---------------------------
root = tk.Tk()
root.title("IHM Contrôle Moteur / Capteurs")
root.geometry("1500x1200")
root.configure(bg="white")

try:
    red_dot = ImageTk.PhotoImage(Image.open(resource_path("red_dot.png")).resize((16, 16)))
    green_dot = ImageTk.PhotoImage(Image.open(resource_path("green_dot.png")).resize((16, 16)))
except Exception as e:
    red_dot = green_dot = None
    print("Warning: Les images de statut n'ont pas été trouvées:", e)
# ------------------------ UI -------------------------------
port_frame = tk.Frame(root, bg="white")
port_frame.pack(fill="x", padx=10, pady=5)

status_label = tk.Label(port_frame, image=red_dot, bg="white")
status_label.image = red_dot
status_label.pack(side="left", padx=5)

auto_start_var = tk.BooleanVar(value=True)
tk.Checkbutton(port_frame, text="Auto Start", variable=auto_start_var, bg='white').pack(side='left')

port_menu = ttk.Combobox(port_frame, state="readonly", width=10)
port_menu.pack(side="left", padx=5)
connexion_button = tk.Button(port_frame, text="Connecter", command=connecter_serial)
connexion_button.pack(side="left", padx=5)
tk.Button(port_frame, text="Rafraîchir", command=actualiser_ports).pack(side="left")
actualiser_ports()

# Zones principales # Haut
top = tk.Frame(root,bg='white'); top.pack(fill='x',padx=10,pady=1, ipady=1)

# Radar # Cadre radar
radf = tk.LabelFrame(top, text="Radar", bg='#f8f9fa')
radf.pack(side='left', expand=False, fill='y', padx=1, ipadx=1, ipady=1)

# Création de la figure radar
fig = plt.Figure(figsize=(4, 4))
radar_ax = fig.add_subplot(111, polar=True)
radar_ax.set_facecolor('black')

# Canvas tkinter pour matplotlib
radar_canvas = FigureCanvasTkAgg(fig, master=radf)
radar_canvas.get_tk_widget().pack()
radar_canvas.draw()
update_radar()

# Cadre Infos
infof = tk.LabelFrame(top, text="Capteurs et Moteur", bg='#20c997')
infof.pack(side='left', expand=False, fill='y', padx=30, ipadx=40, ipady=1)

label_masse = tk.Label(infof, text="Masse : --", bg='white')
label_masse.pack(pady=5)
label_force = tk.Label(infof, text="Force : --", bg='white')
label_force.pack(pady=5)
label_deformation = tk.Label(infof, text="Déformation : --", bg='white')
label_deformation.pack(pady=5)
label_tension = tk.Label(infof, text="Tension : --", bg='white')
label_tension.pack(pady=10)
#status_label.pack(side="left", padx=5)

# Cadre Contrôles DANS Infos
ctl = tk.Frame(infof, bg='#6c757d')
ctl.pack(side='top',  expand=False, fill='y', padx=1, ipadx=1, ipady=1)
tk.Label(ctl, text="Mode:", bg='white').pack(anchor='w')

mode_menu = ttk.Combobox(ctl, values=["Automatique", "Manuel"], state='readonly')
mode_menu.set("Automatique")
mode_menu.pack()
mode_menu.bind("<<ComboboxSelected>>", send_mode)

angle_scale = tk.Scale(ctl, from_=0, to=180, orient='horizontal', label="Angle Manuel", command=lambda v: send_mode())
angle_scale.pack()

angle_label = tk.Label(ctl, text="-- °", bg='white', font=('Arial', 12, 'bold'))
angle_label.pack(pady=5)

# Lame
# Lame 3D
lmf = tk.LabelFrame(top, text="Lame 3D", bg='#f8f9fa')
lmf.pack(side='left', expand=True, fill='both', padx=5, pady=5, ipadx=1, ipady=1)
lame_canvas = tk.Canvas(lmf, width=700, height=400, bg='white', highlightthickness=0)
lame_canvas.pack(fill='both', expand=True, padx=5, pady=5)

# Boutons
bot = tk.Frame(root, bg='white')
bot.pack(pady=10)

start_button = tk.Button(bot, text="Stop", command=toggle_running, bg='red', fg='white', width=10)
start_button.pack(side='left', padx=5)

save_button = tk.Button(bot, text="Sauvegarder", command=save_data, bg='blue', fg='white', width=12)
save_button.pack(side='left', padx=5)

tk.Button(bot, text="Historique", command=show_hist).pack(side='left', padx=5)

tk.Button(bot, text="Reset Graph", bg='#6c757d', command=reset_graph).pack(side='left', padx=5)

tk.Button(bot, text="Reset Zoom", bg='#6c757d', command=reset_zoom).pack(side='left', padx=5)


# Ajoutez ceci dans votre interface (par exemple dans le cadre des boutons)
reset_view_btn = tk.Button(bot, text="Reset Vue 3D", command=reset_3d_view)
reset_view_btn.pack(side='left', padx=5)


chrono_label = tk.Label(bot, text="Temps : 0 s", bg='white')
chrono_label.pack(side='left', padx=20)

# Bas – Graphe
botg = tk.LabelFrame(root, text="Tension", bg='white'); botg.pack(fill='both',expand=True,padx=1,pady=1,ipadx=1,ipady=1)
graph_fig = plt.Figure(figsize=(18,18)); graph_ax=graph_fig.add_subplot(111)
graph_canvas = FigureCanvasTkAgg(graph_fig, master=botg); graph_canvas.get_tk_widget().pack(fill='both', expand=True, ipadx=1, ipady=1); graph_canvas.draw()

# Lancement
update_values()
# === Ajout des connecteurs d'événements ===
# Ajouter juste avant root.mainloop():
graph_canvas.mpl_connect('button_press_event', on_press)
graph_canvas.mpl_connect('button_release_event', on_release)
graph_canvas.mpl_connect('scroll_event', on_scroll)
root.mainloop()

"""pyinstaller --onefile --noconsole --icon=ihm_icon.ico ^
--add-data "red_dot.png;." ^
--add-data "green_dot.png;." ^
IHM_jauge_lame3D.py
"""


# === Fin du code (partie graphique, radar, lame, boutons à réintégrer si besoin) ===
