# build GUI here
from tkinter import *
from tkinter import ttk, messagebox
import time
from waypointalg import subdivide_region
from config import DEFAULT_ALT
from PIL import Image, ImageTk

class GroundStationGUI:
    def __init__(self, root, command_callback, change_port_callback=None):
        self.root = root
        self.command_callback = command_callback
        self.change_port_callback = change_port_callback
        self.drones = {}
        self.selected_drone_id = None
        self.drone_widgets = {}
        self.heartbeat_counts = {}
        self.home_location = None
        self.current_plan = None
        self.region_queue = []
        self.current_region_index = 0

        self.root.title("Drone Mesh Ground Control Station")
        self.root.geometry("900x650")

        self.setup_ui()

    def display_image(self, img):
        """Display a numpy RGB image in the camera panel"""
        pil_img = Image.fromarray(img).resize((256, 256))
        imgtk = ImageTk.PhotoImage(image=pil_img)
        self.cam_label.imgtk = imgtk
        self.cam_label.config(image=imgtk, text='')

    def setup_ui(self):
        main_frame = Frame(self.root)
        main_frame.pack(fill=BOTH, expand=True, padx=10, pady=10)

        # Left panel - Drone list
        left_panel = Frame(main_frame, width=220)
        left_panel.pack(side=LEFT, fill=BOTH, padx=(0, 10))

        Label(left_panel, text="Active Drones", font=("Arial", 12, "bold")).pack()

        self.drone_list_frame = Frame(left_panel)
        self.drone_list_frame.pack(fill=BOTH, expand=True, pady=10)

        Button(left_panel, text="Remove Selected", bg="#f44336", fg="white",
               command=self.remove_selected_drone).pack(fill=X)

        # Right panel
        right_panel = Frame(main_frame)
        right_panel.pack(side=RIGHT, fill=BOTH, expand=True)

        # ---------------- TOP ROW (Telemetry + Camera side by side) ----------------
        top_row = Frame(right_panel)
        top_row.pack(fill=BOTH, expand=True, pady=(0, 10))

        # Telemetry display
        telem_frame = LabelFrame(top_row, text="Telemetry", font=("Arial", 11, "bold"))
        telem_frame.pack(side=LEFT, fill=BOTH, expand=True, padx=(0, 5))

        self.telem_labels = {}
        fields = [
            ('Latitude',   '°'),
            ('Longitude',  '°'),
            ('Altitude',   'm'),
            ('Roll',       '°'),
            ('Pitch',      '°'),
            ('Yaw',        '°'),
            ('Battery',    'V'),
            ('Satellites', ''),
            ('Armed',      ''),
            ('Video',      ''),
            ('Heartbeats', ''),
        ]

        for field, unit in fields:
            row_frame = Frame(telem_frame)
            row_frame.pack(fill=X, padx=10, pady=2)
            Label(row_frame, text=f"{field}:", width=12, anchor=W).pack(side=LEFT)
            value_label = Label(row_frame, text="--", width=15, anchor=W, font=("Courier", 10))
            value_label.pack(side=LEFT)
            Label(row_frame, text=unit, width=5, anchor=W).pack(side=LEFT)
            self.telem_labels[field] = value_label

        # Camera display
        cam_frame = LabelFrame(top_row, text="Camera", font=("Arial", 11, "bold"))
        cam_frame.pack(side=RIGHT, fill=BOTH, expand=True, padx=(5, 0))

        self.cam_label = Label(cam_frame)
        self.cam_label.pack(expand=True)

        self.cam_label.config(text="Waiting for image...", fg="gray")

        bottom_row = Frame(right_panel)
        bottom_row.pack(fill=BOTH, expand=True, pady=(0, 10))

        # Command panel
        cmd_frame = LabelFrame(right_panel, text="Commands", font=("Arial", 11, "bold"))
        cmd_frame.pack(side=LEFT, fill=Y, padx=(0, 5))

        btn_frame = Frame(cmd_frame)
        btn_frame.pack(padx=10, pady=10)

        Button(btn_frame, text="ARM", width=12, bg="#4CAF50", fg="white",
               command=self.cmd_arm).pack(side=LEFT, padx=5)
        Button(btn_frame, text="DISARM", width=12, bg="#f44336", fg="white",
               command=self.cmd_disarm).pack(side=LEFT, padx=5)
        Button(btn_frame, text="Request Image", width=12, bg="#2196F3", fg="white",
               command=self.cmd_request_image).pack(side=LEFT, padx=5)

        wp_frame = Frame(cmd_frame)
        wp_frame.pack(padx=10, pady=(0, 10))

        Label(wp_frame, text="Waypoint:").pack(side=LEFT)
        self.wp_lat = Entry(wp_frame, width=10)
        self.wp_lat.pack(side=LEFT, padx=2)
        self.wp_lat.insert(0, "lat")
        self.wp_lon = Entry(wp_frame, width=10)
        self.wp_lon.pack(side=LEFT, padx=2)
        self.wp_lon.insert(0, "lon")
        self.wp_alt = Entry(wp_frame, width=6)
        self.wp_alt.pack(side=LEFT, padx=2)
        self.wp_alt.insert(0, "alt")
        Button(wp_frame, text="Send Waypoint", bg="#FF9800", fg="white",
               command=self.cmd_waypoint).pack(side=LEFT, padx=5)
        
        map_frame = LabelFrame(right_panel, text="Mini Map", font=("Arial", 11, "bold"))
        map_frame.pack(side=RIGHT, fill=BOTH, expand=True, padx=(5, 0))

        self.map_canvas = Canvas(map_frame, bg="white")
        self.map_canvas.pack(fill=BOTH, expand=True, padx=10, pady=10)
        
    # ------------------------------------------------------------------
    #  Home Location
    # ------------------------------------------------------------------
        home_frame = LabelFrame(cmd_frame, text="Home Location", font=("Arial", 10, "bold"))
        home_frame.pack(padx=10, pady=(5, 5), fill=X)

        inner = Frame(home_frame)
        inner.pack(anchor='center')

        self.home_lat = Entry(inner, width=10)
        self.home_lat.pack(side=LEFT, padx=5)
        self.home_lat.insert(0, "lat")
        self.home_lat.pack(side=LEFT, padx=5)

        self.home_lon = Entry(inner, width=10)
        self.home_lon.pack(side=LEFT, padx=5)
        self.home_lon.insert(0, "lon")
        self.home_lon.pack(side=LEFT, padx=5)

        self.home_alt = Entry(inner, width=6)
        self.home_alt.pack(side=LEFT, padx=5)
        self.home_alt.insert(0, "alt")
        self.home_alt.pack(side=LEFT, padx=5)

        Button(inner,
            text="Set Home",
            bg="#607D8B",
            fg="white",
            command=self.set_home).pack(side=LEFT, padx=8)

    # ------------------------------------------------------------------
    #  Search Region Boundary
    # ------------------------------------------------------------------
        multi_wp_frame = LabelFrame(cmd_frame, text="Search Region Boundary", font=("Arial", 10, "bold"))
        multi_wp_frame.pack(padx=10, pady=10, fill=X)

        self.multi_wp_entries = []

        for i in range(4):
            row = Frame(multi_wp_frame)
            row.pack(pady=2)

            labels = ["Top Left", "Top Right", "Bottom Right", "Bottom Left"]
            Label(row, text=labels[i]).pack(side=LEFT, padx=3)

            lat = Entry(row, width=10)
            lat.insert(0, "lat")
            lat.pack(side=LEFT, padx=2)

            lon = Entry(row, width=10)
            lon.insert(0, "lon")
            lon.pack(side=LEFT, padx=2)

            alt = Entry(row, width=6)
            alt.insert(0, "alt")
            alt.pack(side=LEFT, padx=2)

            self.multi_wp_entries.append((lat, lon, alt))

        Button(multi_wp_frame,
            text="Generate Waypoint Algorithm",
            bg="#9C27B0",
            fg="white",
            command=self.cmd_generate_waypoints).pack(pady=5)
        
        Button(cmd_frame,
            text="Send Waypoint Plan",
            bg="#4CAF50",
            fg="white",
            command=self.cmd_send_plan).pack(pady=5)
        
        Button(cmd_frame,
            text="Next Region",
            bg="#FF5722",
            fg="white",
            command=self.cmd_next_region).pack(pady=3)

        Button(cmd_frame,
            text="Assign Relay → Search",
            bg="#3F51B5",
            fg="white",
            command=self.cmd_assign_relay).pack(pady=3)

        # Bottom bar
        bottom_frame = Frame(self.root)
        bottom_frame.pack(side=BOTTOM, fill=X)

        self.status_label = Label(bottom_frame, text="Ready", bd=1, relief=SUNKEN, anchor=W)
        self.status_label.pack(side=LEFT, fill=X, expand=True)

        Button(bottom_frame, text="Change Port", command=self.on_change_port).pack(
            side=RIGHT, padx=5, pady=2)

    # ------------------------------------------------------------------
    #  Drone list management
    # ------------------------------------------------------------------

    def _add_drone_widget(self, drone_id):
        row = Frame(self.drone_list_frame, relief=GROOVE, bd=1)
        row.pack(fill=X, pady=2, padx=2)

        indicator = Label(row, text="●", fg="green", font=("Arial", 14))
        indicator.pack(side=LEFT, padx=4)

        label = Label(row, text=f"Drone {drone_id}", font=("Courier", 10), cursor="hand2")
        label.pack(side=LEFT, padx=4)

        for widget in (row, label, indicator):
            widget.bind("<Button-1>", lambda e, did=drone_id: self.select_drone(did))

        self.drone_widgets[drone_id] = {
            'frame':     row,
            'indicator': indicator,
            'label':     label,
        }
        self.heartbeat_counts[drone_id] = 0

    def select_drone(self, drone_id):
        bg_default = self.drone_list_frame.cget('bg')
        bg_selected = "#cce5ff"

        for did, w in self.drone_widgets.items():
            w['frame'].config(bg=bg_default)
            w['label'].config(bg=bg_default)
            w['indicator'].config(bg=bg_default)

        if drone_id in self.drone_widgets:
            self.drone_widgets[drone_id]['frame'].config(bg=bg_selected)
            self.drone_widgets[drone_id]['label'].config(bg=bg_selected)
            self.drone_widgets[drone_id]['indicator'].config(bg=bg_selected)

        self.selected_drone_id = drone_id
        if drone_id in self.drones:
            self._update_telemetry_display(self.drones[drone_id])

    def update_drone_indicator(self, drone_id, connected):
        if drone_id in self.drone_widgets:
            self.drone_widgets[drone_id]['indicator'].config(
                fg="green" if connected else "red"
            )

    def remove_selected_drone(self):
        if not self.selected_drone_id:
            messagebox.showwarning("No Drone Selected", "Please select a drone first.")
            return
        self.remove_drone(self.selected_drone_id)

    def remove_drone(self, drone_id):
        if drone_id not in self.drones:
            return
        if drone_id in self.drone_widgets:
            self.drone_widgets[drone_id]['frame'].destroy()
            del self.drone_widgets[drone_id]
        if drone_id in self.heartbeat_counts:
            del self.heartbeat_counts[drone_id]
        del self.drones[drone_id]
        if self.selected_drone_id == drone_id:
            self.selected_drone_id = None
            for label in self.telem_labels.values():
                label.config(text="--", fg="black")
        self.status_label.config(
            text=f"Drone {drone_id} removed | {time.strftime('%H:%M:%S')} | Drones: {len(self.drones)}"
        )

    # ------------------------------------------------------------------
    #  Data updates
    # ------------------------------------------------------------------

    def update_drone(self, drone_data):
        drone_id = drone_data['id']

        if drone_data.get('type') == 'flag':
            self._show_flag_alert(drone_data)
            return

        if drone_data.get('type') == 'photo_start':
            self.status_label.config(
                text=f"Receiving photo from Drone {drone_id} | {time.strftime('%H:%M:%S')}"
            )
            return

        if drone_data.get('type') == 'photo_done':
            self.status_label.config(
                text=f"Photo transfer complete from Drone {drone_id} | {time.strftime('%H:%M:%S')}"
            )
            return

        if drone_id not in self.drones:
            self._add_drone_widget(drone_id)

        self.drones[drone_id] = drone_data
        self.drones[drone_id]['last_seen'] = time.time()

        # increment heartbeat counter
        self.heartbeat_counts[drone_id] = self.heartbeat_counts.get(drone_id, 0) + 1
        count = self.heartbeat_counts[drone_id]

        # update heartbeat in telemetry panel if this drone is selected
        if self.selected_drone_id == drone_id and 'Heartbeats' in self.telem_labels:
            self.telem_labels['Heartbeats'].config(text=str(count))

        self.update_drone_indicator(drone_id, True)

        if self.selected_drone_id == drone_id:
            self._update_telemetry_display(drone_data)

        self.status_label.config(
            text=f"Last update: {time.strftime('%H:%M:%S')} | Drones: {len(self.drones)}"
        )

    def _show_flag_alert(self, data):
        msg = (f"PERSON DETECTED\n\n"
               f"Drone:      {data['id']}\n"
               f"Confidence: {data['confidence']:.0%}\n"
               f"Lat:        {data['lat']:.5f}\n"
               f"Lon:        {data['lon']:.5f}\n"
               f"Alt:        {data['alt']:.1f}m")
        messagebox.showwarning("⚠ Person Detected!", msg)

    def _update_telemetry_display(self, data):
        if data.get('type') == 'telemetry':
            self.telem_labels['Latitude'].config(text=f"{data['lat']:.6f}")
            self.telem_labels['Longitude'].config(text=f"{data['lon']:.6f}")
            self.telem_labels['Altitude'].config(text=f"{data['alt']:.1f}")
            self.telem_labels['Roll'].config(text=f"{data['roll']:.1f}")
            self.telem_labels['Pitch'].config(text=f"{data['pitch']:.1f}")
            self.telem_labels['Yaw'].config(text=f"{data['yaw']:.1f}")
            self.telem_labels['Battery'].config(text=f"{data['battery']:.2f}")
            self.telem_labels['Satellites'].config(text=str(data['satellites']))
            self.telem_labels['Armed'].config(
                text="YES" if data['armed'] else "NO",
                fg="green" if data['armed'] else "red"
            )
            self.telem_labels['Video'].config(
                text="STREAMING" if data.get('streaming') else "OFF",
                fg="blue" if data.get('streaming') else "gray"
            )

    def draw_waypoints_on_map(self, coords, home=None, relay=None, center=None, subregions=None):
        self.map_canvas.delete("all")

        if not coords:
            return

        # extract lists
        lats = [c[0] for c in coords]
        lons = [c[1] for c in coords]

        if home:
            lats.append(home[0])
            lons.append(home[1])

        if center:
            lats.append(center[0])
            lons.append(center[1])

        if relay:
            for lat, lon in relay:
                lats.append(lat)
                lons.append(lon)

        min_lat, max_lat = min(lats), max(lats)
        min_lon, max_lon = min(lons), max(lons)

        width = self.map_canvas.winfo_width()
        height = self.map_canvas.winfo_height()

        if width < 10 or height < 10:
            self.root.after(100, lambda: self.draw_waypoints_on_map(coords, home, relay, center))            
            return

        margin = 0.05

        import math

        avg_lat = sum(lats) / len(lats)

        def transform(lat, lon):
            # scale longitude correctly
            x = (lon - min_lon) * math.cos(math.radians(avg_lat))
            x = x / ((max_lon - min_lon) * math.cos(math.radians(avg_lat)) + 1e-9)

            y = (lat - min_lat) / (max_lat - min_lat + 1e-9)
            y = 1 - y

            # margin
            margin = 0.05
            x = margin + (1 - 2*margin) * x
            y = margin + (1 - 2*margin) * y

            return int(x * width), int(y * height)
        
        def safe_text(x, y, text, color):
            x = max(20, min(x, width - 60))
            y = max(10, min(y, height - 10))

            self.map_canvas.create_text(x, y, text=text, fill=color, anchor=W)

        points = []
        for c in coords:
            lat, lon = c[0], c[1]
            points.append(transform(lat, lon))

        # draw lines
        for i in range(len(points) - 1):
            self.map_canvas.create_line(
                points[i][0], points[i][1],
                points[i+1][0], points[i+1][1],
                fill="cyan", width=2
            )

        # draw points
        for i, (x, y) in enumerate(points):
            self.map_canvas.create_oval(x-4, y-4, x+4, y+4, fill="red")
            safe_text(x+8, y, f"P{i+1}", "white")

        if home:
            hx, hy = transform(home[0], home[1])
            self.map_canvas.create_oval(hx-6, hy-6, hx+6, hy+6, fill="red")
            safe_text(hx+10, hy, "HOME", "red")
    
        if center:
            cx, cy = transform(center[0], center[1])
            self.map_canvas.create_oval(cx-6, cy-6, cx+6, cy+6, fill="green")
            safe_text(cx+10, cy, "SEARCH", "green")

        if relay:
            for i, (lat, lon) in enumerate(relay):
                rx, ry = transform(lat, lon)
                self.map_canvas.create_oval(rx-4, ry-4, rx+4, ry+4, fill="blue")
                safe_text(rx+8, ry, f"R{i}", "blue")

        if subregions:
            for sub in subregions:
                sub_points = [transform(lat, lon) for lat, lon in sub]

                for i in range(len(sub_points)):
                    self.map_canvas.create_line(
                        sub_points[i][0], sub_points[i][1],
                        sub_points[(i+1) % len(sub_points)][0], sub_points[(i+1) % len(sub_points)][1],
                        fill="#888888",  # faint gray
                        width=2
                    )

        # draw bounding box for region
        for i in range(len(points)):
            self.map_canvas.create_line(
                points[i][0], points[i][1],
                points[(i+1) % len(points)][0], points[(i+1) % len(points)][1],
                fill="cyan"
            )

    # ------------------------------------------------------------------
    #  Commands
    # ------------------------------------------------------------------

    def on_change_port(self):
        if self.change_port_callback:
            self.change_port_callback()

    def cmd_arm(self):
        if self.selected_drone_id:
            self.command_callback('arm', self.selected_drone_id)
        else:
            messagebox.showwarning("No Drone Selected", "Please select a drone first.")

    def cmd_disarm(self):
        if self.selected_drone_id:
            self.command_callback('disarm', self.selected_drone_id)
        else:
            messagebox.showwarning("No Drone Selected", "Please select a drone first.")

    def cmd_request_image(self):
        if self.selected_drone_id:
            self.command_callback('request_image', self.selected_drone_id)
        else:
            messagebox.showwarning("No Drone Selected", "Please select a drone first.")

    def cmd_waypoint(self):
        if not self.selected_drone_id:
            messagebox.showwarning("No Drone Selected", "Please select a drone first.")
            return
        try:
            lat = float(self.wp_lat.get())
            lon = float(self.wp_lon.get())
            alt = float(self.wp_alt.get())
            self.command_callback('waypoint', self.selected_drone_id, lat, lon, alt)
        except ValueError:
            messagebox.showerror("Invalid Input", "Please enter valid numeric coordinates.")

    def set_home(self):
        try:
            lat = float(self.home_lat.get())
            lon = float(self.home_lon.get())
            alt = float(self.home_alt.get())

            self.home_location = (lat, lon, alt)

            self.status_label.config(
                text=f"Home set: {lat:.5f}, {lon:.5f}, {alt:.1f}m"
            )

            print("Home location set:", self.home_location)

        except ValueError:
            messagebox.showerror("Invalid Input", "Enter valid home coordinates.")

    def cmd_generate_waypoints(self):
        if not self.home_location:
            messagebox.showwarning("No Home Set", "Please set a home location first.")
            return

        coords = []
        try:
            for lat_e, lon_e, alt_e in self.multi_wp_entries:
                lat = float(lat_e.get())
                lon = float(lon_e.get())
                alt = float(alt_e.get())
                coords.append((lat, lon, alt))
        except ValueError:
            messagebox.showerror("Invalid Input", "Enter valid numbers.")
            return

        coords_sorted = sorted(coords, key=lambda c: c[0], reverse=True)
        top = sorted(coords_sorted[:2], key=lambda c: c[1])
        bottom = sorted(coords_sorted[2:], key=lambda c: c[1])

        ordered = [
            top[0], top[1],
            bottom[1], bottom[0]
        ]

        home = (self.home_location[0], self.home_location[1])

        from waypointalg import relay_search_drones
        ordered_2d = [(lat, lon) for lat, lon, _ in ordered]
        
        print("INPUT TO ALGO:")
        print("TL:", ordered_2d[0])
        print("TR:", ordered_2d[1])
        print("BR:", ordered_2d[2])
        print("BL:", ordered_2d[3])
        print("HOME:", home)

        relay, center, flag = relay_search_drones(
            ordered_2d[0],
            ordered_2d[1],
            ordered_2d[2],
            ordered_2d[3],
            home
        )

        regions = subdivide_region(
            ordered_2d[0],
            ordered_2d[1],
            ordered_2d[2],
            ordered_2d[3]
        )

        def region_center(region):
            tl, tr, br, bl = region
            return (
                (tl[0] + tr[0] + br[0] + bl[0]) / 4,
                (tl[1] + tr[1] + br[1] + bl[1]) / 4
            )

        def distance(a, b):
            from waypointalg import dist
            return dist(a, b)

        home_2d = (self.home_location[0], self.home_location[1])

        self.region_queue = sorted(
            regions,
            key=lambda r: distance(home_2d, region_center(r))
        )

        self.draw_waypoints_on_map(
            ordered,
            home=(self.home_location[0], self.home_location[1]),
            relay=relay,
            center=center,
            subregions=self.region_queue
        )

        print("\n=== SORTED REGION QUEUE ===")
        for i, r in enumerate(self.region_queue):
            c = region_center(r)
            d = distance(home_2d, c)
            print(f"Region {i}: center={c}, dist={d:.1f} m")

        self.current_region_index = 0
        self.current_region_index = 0

        print("\n=== REGION QUEUE ===")
        for i, r in enumerate(regions):
            tl, tr, br, bl = r
            center = (
                (tl[0]+tr[0]+br[0]+bl[0])/4,
                (tl[1]+tr[1]+br[1]+bl[1])/4
            )
            print(f"Region {i}: center={center}")

        if self.region_queue:
            region = self.region_queue[0]

            relay, center, flag = relay_search_drones(
                region[0], region[1], region[2], region[3], home
            )
        else:
            relay, center, flag = [], None, 1

        self.current_plan = {
            "corners": ordered,
            "relay": relay,
            "center": center
        }

        self.draw_waypoints_on_map(
            ordered,
            home=(self.home_location[0], self.home_location[1]),
            relay=relay,
            center=center
        )

        print("PLAN GENERATED:")
        print(self.current_plan)

    def cmd_send_plan(self):
        if not self.selected_drone_id:
            messagebox.showwarning("No Drone Selected", "Select a drone to send plan.")
            return

        if not self.current_plan:
            messagebox.showwarning("No Plan", "Generate a plan first.")
            return

        drone_id = self.selected_drone_id

        center = self.current_plan["center"]

        if center:
            lat, lon = center
            alt = DEFAULT_ALT

            self.command_callback('waypoint', drone_id, lat, lon, alt)

        for wp in self.current_plan["relay"]:
            lat, lon = wp
            alt = DEFAULT_ALT

            self.command_callback('waypoint', drone_id, lat, lon, alt)

        print(f"Sent plan to Drone {drone_id}")

    def cmd_next_region(self):
        if not self.region_queue:
            messagebox.showwarning("No Regions", "Generate regions first.")
            return

        if self.current_region_index >= len(self.region_queue) - 1:
            messagebox.showinfo("Done", "All regions processed.")
            return

        self.current_region_index += 1

        region = self.region_queue[self.current_region_index]

        home = (self.home_location[0], self.home_location[1])

        from waypointalg import relay_search_drones

        relay, center, flag = relay_search_drones(
            region[0], region[1], region[2], region[3], home
        )

        self.current_plan["relay"] = relay
        self.current_plan["center"] = center

        print(f"\n=== MOVED TO REGION {self.current_region_index} ===")
        print("Center:", center)
        print("Relay:", relay)

        self.draw_waypoints_on_map(
            self.current_plan["corners"],
            home=(self.home_location[0], self.home_location[1]),
            relay=relay,
            center=center,
            subregions=self.region_queue
        )

    def cmd_assign_relay(self):
        if not self.current_plan:
            messagebox.showwarning("No Plan", "Generate a plan first.")
            return

        relay = self.current_plan.get("relay", [])

        if not relay:
            print("No relay drones available")
            return

        new_search = relay[-1]

        print("\n=== RELAY HANDOFF ===")
        print("New search position:", new_search)

        if self.selected_drone_id:
            lat, lon = new_search
            self.command_callback('waypoint', self.selected_drone_id, lat, lon, DEFAULT_ALT)