import obj_files_handler as obj_files_handler 
from geometry import Geometry
from timer import time_me

import math
import tkinter as tk
from tkinter import ttk, filedialog, colorchooser, messagebox
# import pyscreenshot as ImageGrab

import sv_ttk
import os

MIN_WIDTH = 1165
MIN_HEIGHT = 630

PLOTTING_STATION = 0
REL_PATH = "App_python/src/"

class GUI(tk.Tk):
	''''''
	CANVAS_WIDTH = 800
	CANVAS_HEIGHT = 570
	CANVAS_COLOR = 'white'
	COMMON_X = 0.98	# Many graphical elements share the same relative X position
	MOVING_STEP = 10

	SEPARATIONX = 0.06
	SEPARATIONY = 0.08
	ORIGINY = 0.34
	
	POINT_SIZE = 1 
	POINT_COLOR = '#131313'

	def __init__(self, title='GUI FACHA - GRUPO 4', min_size=(MIN_WIDTH, MIN_HEIGHT)):
		''''''
		super().__init__()
		# Set the theme to be dark (there must be an initialized app)
		sv_ttk.set_theme("dark")

		icon_path = REL_PATH+"icon.bmp" 
		icon_ico = REL_PATH+"icon.ico"
		# self.iconbitmap(bitmap=icon_path, default=icon_ico)
		
		self._file_exists = False # A flag for whether the file has been loaded or not
		self._changed = True # A flag used to only redraw the object when a change occured
		self._geometry_handler = Geometry(self.CANVAS_WIDTH, self.CANVAS_HEIGHT)
		
		# Initial canvas ratios
		self._canvas_w = int((self.CANVAS_WIDTH/MIN_WIDTH)*MIN_WIDTH)
		self._canvas_h = int((self.CANVAS_HEIGHT/MIN_HEIGHT)*MIN_HEIGHT)
		
		self._fill_color_holder = "#000000"
		self._line_color_holder = "#0000FF"

		self.x_rotation_plot = 0
		self.y_rotation_plot = 0
		self.z_rotation_plot = 0

		self.data_matrix = [[0 for x in range(4)] for y in range(7)]

		self.__initialise_window(title, min_size)
		self.__create_widgets()
		self.__reset_rotation()

	def __initialise_window(self, title, min_size):
		self.title(title)
		self.minsize(*min_size)
		
	def __create_widgets(self):
		self.__create_canvas()
		self.__create_zoom_slider()
		self.__create_rotation_value_boxes()
		self.__create_import_file_button()
		self.__create_color_pickers()
		self.__create_fill_check()
		self.__create_station_buttons()
		self.__create_cpu_usage_monitor()

	def __create_canvas(self):
		self._canvas_color = tk.StringVar()
		self._canvas_color.set("#FFFFFF")
		self._canvas = tk.Canvas(self, bg=self._canvas_color.get())
		self._canvas.place(relx=0.01, rely=0.01, relwidth=self.CANVAS_WIDTH/MIN_WIDTH, relheight=self.CANVAS_HEIGHT/MIN_HEIGHT)
		# Catch the canvas resize event
		self._canvas.bind("<Configure>", self.__resized)

		# Bind the scrolling on canvas event to zoom slider (Button-4/5 on linux)
		self._canvas.bind('<Button-4>', self.__mousewheel_scroll_in_canvas_up_event)
		self._canvas.bind('<Button-5>', self.__mousewheel_scroll_in_canvas_down_event)

	def __create_zoom_slider(self):
		ttk.Label(self, text="Zoom:").place(relx=0.65, rely=0.92, relheight=0.035, relwidth=0.1, anchor="ne")
		self._zoom_slider = ttk.Scale(self, from_=0.1, to=99.9, orient="horizontal", command=self.__changed)
		self._zoom_slider.set(self._geometry_handler.zoom)
		self._zoom_slider.place(relx=0.7, rely=0.92, relheight=0.04, relwidth=0.1, anchor="ne")

	def __create_reset_rot_button(self):
		ttk.Button(self, text="Reset Rot", command=self.__reset_rotation).place(relx=0.8, rely=0.92, relheight=0.05, relwidth=0.095, anchor="ne")

	def __create_import_file_button(self):
		self._file_name = tk.StringVar()
		ttk.Label(self, textvariable=self._file_name, foreground="#AAAAAA").place(relx=0.01, rely=0.96, relheight=0.035, relwidth=0.4)
		ttk.Button(self, text="Import Model", command=self.__read_file).place(relx=self.COMMON_X, rely=0.92, relheight=0.05, relwidth=0.1, anchor="ne")

	def __create_station_buttons(self):
		for i in range(0, 7):
			ttk.Button(self, text="Station " + str(self.stations[i]), \
				command=lambda i=i: self.__plotting_station(i)).place(relx=self.COMMON_X-(3*self.SEPARATIONX), \
					rely=self.ORIGINY+(i*self.SEPARATIONY), relheight=0.05, relwidth=0.095, anchor="ne")
			
	def __create_cpu_usage_monitor(self):
		self.cpu_usage = ttk.Label(self, text="CPU Usage: 0%")
		self.cpu_usage.place(relx=self.COMMON_X-(4.5*self.SEPARATIONX), rely=self.ORIGINY-4*(self.SEPARATIONY), relheight=0.04, relwidth=0.27, anchor="nw")
		self.mem_usage = ttk.Label(self, text="MEM Usage: 0%")
		self.mem_usage.place(relx=self.COMMON_X-(4.5*self.SEPARATIONX), rely=self.ORIGINY-3*(self.SEPARATIONY), relheight=0.04, relwidth=0.27, anchor="nw")


	def __create_rotation_value_boxes(self):
		self.x_rot_value = [None, None, None, None, None, None, None]
		self.y_rot_value = [None, None, None, None, None, None, None]
		self.z_rot_value = [None, None, None, None, None, None, None]
		self.stations = [1, 2, 3, 4, 5, 6, 7]
		self.station_label  = [None, None, None, None, None, None, None]
		#self.x_rot_label = ttk.Label(self, text="X Rotation:")
		#self.x_rot_label.place(relx=self.COMMON_X, rely=0.485, relheight=0.035, relwidth=0.075, anchor="ne")

		R_tag = ttk.Label(self, text="R")
		R_tag.place(relx=self.COMMON_X, rely=self.ORIGINY-(self.SEPARATIONY), relheight=0.04, relwidth=0.05, anchor="ne")
	
		C_tag = ttk.Label(self, text="C")
		C_tag.place(relx=self.COMMON_X-(1*self.SEPARATIONX), rely=self.ORIGINY-(self.SEPARATIONY), relheight=0.04, relwidth=0.05, anchor="ne")
	
		O_tag = ttk.Label(self, text="O")
		O_tag.place(relx=self.COMMON_X-(2*self.SEPARATIONX), rely=self.ORIGINY-(self.SEPARATIONY), relheight=0.04, relwidth=0.05, anchor="ne")

		self.P_tag = ttk.Label(self, text=str(PLOTTING_STATION))
		self.P_tag.place(relx=self.COMMON_X-(3*self.SEPARATIONX), rely=self.ORIGINY-(self.SEPARATIONY), relheight=0.04, relwidth=0.05, anchor="ne")

		for i in range(0, 7):
			self.x_rot_value[i] = ttk.Label(self, text=str(0))
			self.x_rot_value[i].place(relx=self.COMMON_X-(0*self.SEPARATIONX), rely=self.ORIGINY+(i*self.SEPARATIONY), relheight=0.04, relwidth=0.05, anchor="ne")

			self.y_rot_value[i] = ttk.Label(self, text=str(0))
			self.y_rot_value[i].place(relx=self.COMMON_X-(1*self.SEPARATIONX), rely=self.ORIGINY+(i*self.SEPARATIONY), relheight=0.04, relwidth=0.05, anchor="ne")

			self.z_rot_value[i] = ttk.Label(self, text=str(0))
			self.z_rot_value[i].place(relx=self.COMMON_X-(2*self.SEPARATIONX), rely=self.ORIGINY+(i*self.SEPARATIONY), relheight=0.04, relwidth=0.05, anchor="ne")

			


	def update_rotation_values(self):
		for i in range(0, 7):
			self.x_rot_value[i].config(text=str(self.data_matrix[i][0])[:5])
			self.y_rot_value[i].config(text=str(self.data_matrix[i][1])[:5])
			self.z_rot_value[i].config(text=str(self.data_matrix[i][2])[:5])

		global PLOTTING_STATION
		self.P_tag.config(text=str(PLOTTING_STATION+1))
		self.x_rotation_plot = float(self.x_rot_value[PLOTTING_STATION].cget("text"))
		self.y_rotation_plot = float(self.y_rot_value[PLOTTING_STATION].cget("text"))
		self.z_rotation_plot = float(self.z_rot_value[PLOTTING_STATION].cget("text"))
	
	def update_cpu_usage(self, cpu_usage, mem_usage):
		bars = 20
		cpu_percent = cpu_usage/100
		mem_percent = mem_usage/100
		cpu_bars = '█' * int(cpu_percent * bars) + '-' * (bars - int(cpu_percent * bars))
		mem_bars = '█' * int(mem_percent * bars) + '-' * (bars - int(mem_percent * bars))
		self.cpu_usage.config(text="CPU Usage: "+cpu_bars+str(cpu_usage)[:3]+"%")
		self.mem_usage.config(text="MEM Usage: "+mem_bars+str(mem_usage)[:3]+"%")


	def on_x_rot_slider_change(self, value):
		self.update_rotation_values(self.x_rot_slider.get(), self.y_rot_slider.get(), self.z_rot_slider.get())

	def on_y_rot_slider_change(self, value):
		self.update_rotation_values(self.x_rot_slider.get(), self.y_rot_slider.get(), self.z_rot_slider.get())

	def on_z_rot_slider_change(self, value):
		self.update_rotation_values(self.x_rot_slider.get(), self.y_rot_slider.get(), self.z_rot_slider.get())

	def __plotting_station(self, value):
		global PLOTTING_STATION
		PLOTTING_STATION = value
		print(PLOTTING_STATION)
		self.__changed()

	def __create_color_pickers(self):
		'''Create the color pickers to change line/canvas/fill colors'''
		COMM_Y = 0.935

		# FILL
		self.fill_color = tk.StringVar()
		self.fill_color.set("#000000")
		ttk.Label(self, text="Fill color:").place(relx=0.01, rely=COMM_Y-0.01, relheight=0.035)
		self._fill_btn = tk.Button(self, text="", command=self.__pick_color_fill, relief='flat')
		self._fill_btn.place(relx=0.08, rely=COMM_Y, relheight=0.015, relwidth=0.05)
		self._fill_btn['bg'] = self.fill_color.get()

		# LINE
		self.line_color = tk.StringVar()
		self.line_color.set("#0000FF")
		ttk.Label(self, text="Line color:").place(relx=0.2, rely=COMM_Y-0.01, relheight=0.035, anchor="ne")
		self._line_btn = tk.Button(self, text="", command=self.__pick_color_line, relief='flat')
		self._line_btn.place(relx=0.21, rely=COMM_Y, relheight=0.015, relwidth=0.05)
		self._line_btn['bg'] = self.line_color.get()

		# CANVAS' BACKGROUND
		ttk.Label(self, text="Canvas color:").place(relx=0.35, rely=COMM_Y - 0.01, relheight=0.035, anchor="ne")
		self._canvas_btn = tk.Button(self, text="", command=self.__pick_color_canvas, relief='flat')
		self._canvas_btn.place(relx=0.36, rely=COMM_Y, relheight=0.015, relwidth=0.05)
		self._canvas_btn['bg'] = self._canvas_color.get()

	def __create_fill_check(self):
		self._check_no_fill = tk.IntVar()
		ttk.Checkbutton(self, text="No fill", variable=self._check_no_fill, command=self.__changed, onvalue=True, offvalue=False).place(relx=0.80, rely=0.92)

	def __pick_color_fill(self):
		self.__pick_color("f")

	def __pick_color_line(self):
		self.__pick_color("l")

	def __pick_color_canvas(self):
		self.__pick_color("c")

	def __pick_color(self, picker):
		if(picker == "f"):
			col = colorchooser.askcolor(initialcolor = self.fill_color.get())
			if(col[1]):
				self.fill_color.set(col[1])
				self._fill_btn['bg']  = col[1]
		
		elif(picker == "c"):
			col = colorchooser.askcolor(initialcolor = self._canvas_color.get())
			if(col[1]):
				self._canvas_color.set(col[1])
				self._canvas_btn['bg']  = col[1]
				self._canvas['bg'] = self._canvas_color.get()
		
		else:
			col = colorchooser.askcolor(initialcolor = self.line_color.get())
			if(col[1]):
				self.line_color.set(col[1])
				self._line_btn['bg']  = col[1]
		
		self.__changed()

	def __mousewheel_scroll_in_canvas_up_event(self, *args):
		'''callback to the scrolling up in canvas event'''
		self._zoom_slider.set(self._zoom_slider.get()-0.5)

	def __mousewheel_scroll_in_canvas_down_event(self, *args):
		'''callback to the scrolling down in canvas event'''
		self._zoom_slider.set(self._zoom_slider.get()+0.5)

	def __get_canvas_shape(self):
		"""returns the shape of the canvas holding the visualized frame"""
		self.update()
		return self._canvas.winfo_width(), self._canvas.winfo_height()

	def __resized(self, *args):
		'''Callback to the window resize events'''
		w, h = self.__get_canvas_shape()
		if self._canvas_w != w or self._canvas_h != h:
			# Keep the object in the middle of the canvas
			self._geometry_handler.update_position((w-self._canvas_w)//2, (h-self._canvas_h)//2)
			self._canvas_w = w
			self._canvas_h = h
			self.__changed()

	def __changed(self, *args):
		'''Signal to the rendering function that something has changed in the object'''
		self._changed = True

	def __reset_rotation(self):
		self._geometry_handler.reset_rotation()
		self.__changed()

	def __take_screenshot(self):
		#Grab the top-left corner's coordinates
		x = self.winfo_rootx() + self._canvas.winfo_x()
		y = self.winfo_rooty() + self._canvas.winfo_y()

		#Grab the bottom-right corner's coordinates
		x1 = x + self._canvas.winfo_width()
		y1 = y + self._canvas.winfo_height()

		#Crop the screenshot to the determined coordinates
		screenshot = ImageGrab.grab().crop((x, y, x1, y1))
		save_path = filedialog.asksaveasfilename(defaultextension=".png", filetypes=(("PNG Files", "*.png"), ("All Files", "*.*")))

		#Check if they actually saved and didn't exit before saving
		if save_path:
			screenshot.save(save_path)

	def __read_file(self):
		messagebox.showinfo(message='Only .obj files are compatible!', title="WARNING")

		file_path = filedialog.askopenfilename(defaultextension=".obj", filetypes=(("OBJ Files", "*.obj"), ("All Files", "*.*")))

		if len(file_path) and file_path[-4:] != ".obj":
			messagebox.showinfo(message="Incompatible file format", title="ERROR")

		elif len(file_path):
			self._file_name.set(file_path.split('/')[-1])
			self.__reset_rotation()
			with open(file_path) as file:
				self._geometry_handler.upload_object(*obj_files_handler.extract_data(file))
				self._file_exists = True

#	@time_me
	def render(self):
		'''Render the object on the screen'''
		self.update_rotation_values()
		self.__reset_rotation()
		self.__set_rotations()
		self.__set_zoom()

		if self._file_exists and (self._changed):
			#Delete all the previous points and lines in order to draw new ones
			self._canvas.delete("all")
			self.__update_colors()
			self.__draw_object()
			self._changed = False

	def __set_zoom(self):
		self._geometry_handler.set_zoom(self._zoom_slider.get())

	
	def __set_rotations(self):
		'''Set the required rotations for the geometry handler'''
		x, y, z = self._geometry_handler.orientation

		self._geometry_handler.reset_rotation(
			x=self.x_rotation_plot, 
			y=self.y_rotation_plot, 
			z=self.z_rotation_plot
		)

	def __change_fill_color(self, color: str, no_fill: bool = False):
		'''Change the face fill color'''
		self._fill_color_holder = "" if no_fill else color

	def __change_line_color(self, color):
		''''''
		self._line_color_holder = color

	def __draw_point(self, point: 'tuple(int, int)') -> None:
		'''Draw a point on the canvas'''
		self._canvas.create_oval(point[0], point[1],
						   		 point[0], point[1],
						   		 width=self.POINT_SIZE,
						   		 fill=self.POINT_COLOR)

	@time_me
	def __draw_faces(self, points: dict) -> None:
		''''''
		for face in self._geometry_handler.faces:
			# Grab the points that make up that specific face
			to_draw = [points[f] for f in face]
			
			self._canvas.create_polygon(to_draw, outline=self._line_color_holder, fill=self._fill_color_holder)
	
	def __draw_object(self):
		'''Draw the object on the canvas'''
		projected_points = self._geometry_handler.transform_object()
		self.__draw_faces(projected_points)
	
	def __update_colors(self):
		self.__change_fill_color(self.fill_color.get(), self._check_no_fill.get())
		self.__change_line_color(self.line_color.get())
